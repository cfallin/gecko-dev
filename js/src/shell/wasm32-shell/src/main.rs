/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

use std::sync::Once;
use wasi_common::WasiCtx;
use wasmtime::*;
use wasmtime_wasi::WasiCtxBuilder;

static INITIALIZE_JIT_RUNTIME: Once = Once::new();

#[cfg(debug_assertions)]
use std::io::prelude::*;

#[cfg(debug_assertions)]
use std::sync::atomic::{AtomicUsize, Ordering};

#[cfg(debug_assertions)]
static GLOBAL_JIT_MODULE_COUNT: AtomicUsize = AtomicUsize::new(0);

fn call_instance_function(
    instance: &mut Instance,
    function_name: &str,
    params: &[Val],
    results: &mut [Val],
    store: &mut Store<wasi_common::WasiCtx>,
) -> Result<()> {
    let function = instance
        .get_func(&mut *store, function_name)
        .unwrap_or_else(|| panic!("can't find {} function", function_name));
    function.call(&mut *store, params, results)
}

fn write_string(
    inst: &mut Instance,
    string: String,
    store: &mut Store<wasi_common::WasiCtx>,
) -> i32 {
    // ptr = AllocateBytes()
    let mut results: [Val; 1] = [Val::I32(0)];
    call_instance_function(
        &mut *inst,
        "AllocateBytes",
        &[Val::I32((string.len() as i32) + 1)],
        &mut results,
        store,
    )
    .expect("can't call AllocateBytes");
    let ptr = results[0].i32().unwrap();

    // fill memory with utf-8 bytes
    let memory = inst
        .get_memory(&mut *store, "memory")
        .expect("there is no exported memory there");
    let c_str = std::ffi::CString::new(string).unwrap();
    memory
        .write(&mut *store, ptr as usize, c_str.as_bytes_with_nul())
        .expect("mem.write return error");

    ptr
}

fn install_jit_module(name: &str, caller: &mut Caller<'_, WasiCtx>) -> Result<()> {
    // Call jitModule or jitRuntimeModule.
    let jit_module_func = match caller.get_export(name) {
        Some(Extern::Func(func)) => func,
        _ => anyhow::bail!("failed to find jitModule"),
    };

    let jit_module_ptr = {
        let mut results: [Val; 1] = [Val::I32(0)];
        jit_module_func
            .call(&mut *caller, &[], &mut results)
            .expect("failed to call jitModule");
        results[0].i32().unwrap()
    };

    // Call moduleData.
    let jit_module_data_ptr = {
        let module_data_func = match caller.get_export("moduleData") {
            Some(Extern::Func(func)) => func,
            _ => anyhow::bail!("failed to find moduleData"),
        };

        let mut results: [Val; 1] = [Val::I32(0)];
        module_data_func
            .call(
                &mut *caller,
                &[wasmtime::Val::I32(jit_module_ptr)],
                &mut results,
            )
            .expect("failed to call moduleData");
        results[0].i32().unwrap()
    };

    let jit_module_size = {
        let module_size_func = match caller.get_export("moduleSize") {
            Some(Extern::Func(func)) => func,
            _ => anyhow::bail!("failed to find moduleSize"),
        };

        let mut results: [Val; 1] = [Val::I32(0)];
        module_size_func
            .call(
                &mut *caller,
                &[wasmtime::Val::I32(jit_module_ptr)],
                &mut results,
            )
            .expect("failed to call moduleSize");
        results[0].i32().unwrap()
    };

    // Copy data from inst.memory into rust array.
    let memory = match caller.get_export("memory") {
        Some(Extern::Memory(mem)) => mem,
        _ => anyhow::bail!("failed to find host memory"),
    };

    let mut bytes = vec![0 as u8; jit_module_size as usize];
    memory
        .read(&mut *caller, jit_module_data_ptr as usize, &mut bytes)
        .expect("can't read Wasm bytes from memory");

    // Define exports.
    let function_table = match caller.get_export("__indirect_function_table") {
        Some(Extern::Table(table)) => table,
        _ => anyhow::bail!("failed to find the host function table"),
    };

    let stack_pointer_global = match caller.get_export("__stack_pointer") {
        Some(Extern::Global(global)) => global,
        _ => anyhow::bail!("failed to find the host stack pointer global"),
    };

    let mut linker = Linker::new(&caller.engine());
    linker
        .define(&mut *caller, "env", "memory", memory)
        .expect("can't define memory in linker");
    linker
        .define(
            &mut *caller,
            "env",
            "__indirect_function_table",
            function_table,
        )
        .expect("can't define function table in linker");
    linker
        .define(&mut *caller, "env", "__stack_pointer", stack_pointer_global)
        .expect("can't define stack pointer global in linker");

    // Instantiate Wasm module from bytes.
    let jit_module = Module::from_binary(caller.engine(), bytes.as_slice())
        .expect("can't create a module from Wasm bytes");
    linker
        .instantiate(&mut *caller, &jit_module)
        .expect("can't instantiate jit module");

    #[cfg(debug_assertions)]
    {
        let id = GLOBAL_JIT_MODULE_COUNT.fetch_add(1, Ordering::SeqCst);
        let mut dump_file_name = id.to_string();
        dump_file_name.push_str(".wasm");
        let mut file = std::fs::File::create(&dump_file_name).expect("can't dump wasm module");
        file.write_all(bytes.as_slice())
            .expect("can't dump jit module");
    }

    // Free module.
    let free_module_func = match caller.get_export("freeModule") {
        Some(Extern::Func(func)) => func,
        _ => anyhow::bail!("failed to find freeModule"),
    };
    free_module_func
        .call(&mut *caller, &[wasmtime::Val::I32(jit_module_ptr)], &mut [])
        .expect("failed to call freeModule");

    Ok(())
}

fn main() -> wasmtime::Result<()> {
    let engine = Engine::default();
    let mut linker = Linker::new(&engine);
    wasmtime_wasi::add_to_linker(&mut linker, |s| s)?;

    let mut tests_path =
        std::env::var("WASM32_TESTS_PATH").expect("could't find WASM32_TESTS_PATH env variable");
    let mut jit_tests_path = tests_path.clone();
    jit_tests_path.push_str("/jit-test/");
    tests_path.push_str("/tests/");

    let wasi = WasiCtxBuilder::new()
        .inherit_stdio()
        .inherit_stdout()
        .inherit_stderr()
        .preopened_dir(
            wasmtime_wasi::Dir::open_ambient_dir(
                std::path::Path::new("."),
                wasmtime_wasi::ambient_authority(),
            )?,
            ".",
        )?
        .preopened_dir(
            wasmtime_wasi::Dir::open_ambient_dir(
                std::path::Path::new(&tests_path),
                wasmtime_wasi::ambient_authority(),
            )?,
            &tests_path,
        )?
        .preopened_dir(
            wasmtime_wasi::Dir::open_ambient_dir(
                std::path::Path::new(&jit_tests_path),
                wasmtime_wasi::ambient_authority(),
            )?,
            &jit_tests_path,
        )?
        .build();
    let mut store = Store::new(&engine, wasi);

    let main_js_module =
        std::env::var("WASM32_MAIN_SHELL").expect("could't find WASM32_MAIN_SHELL env variable");
    let module = Module::from_file(&engine, main_js_module)?;

    // Define env::CompileCallbackForTests
    linker.func_wrap(
        "env",
        "CompileCallbackForTests",
        |mut caller: Caller<'_, WasiCtx>| -> Result<()> {
            INITIALIZE_JIT_RUNTIME.call_once(|| {
                install_jit_module("jitRuntimeModule", &mut caller)
                    .expect("can't install jit runtime");
            });

            install_jit_module("jitModule", &mut caller)
        },
    )?;

    let mut instance = linker.instantiate(&mut store, &module)?;
    call_instance_function(&mut instance, "_initialize", &[], &mut [], &mut store)?;

    let argv_ptr = {
        // Call AllocateBytes to allocate bytes for argv.
        let mut results: [Val; 1] = [Val::I32(0)];
        call_instance_function(
            &mut instance,
            "AllocateBytes",
            &[Val::I32((std::env::args().len() as i32) * 4)],
            &mut results,
            &mut store,
        )?;
        results[0].i32().unwrap()
    };

    // Copy argv.
    let mut i = 0;
    let args: Vec<String> = std::env::args().collect();
    for str in args {
        // Allocate string.
        let arg_ptr = write_string(&mut instance, str, &mut store);

        // Call SetArgv.
        call_instance_function(
            &mut instance,
            "SetArgv",
            &[Val::I32(argv_ptr), Val::I32(i as i32), Val::I32(arg_ptr)],
            &mut [],
            &mut store,
        )?;
        i += 1;
    }

    let main_return_code = {
        // Call main.
        let mut results: [Val; 1] = [Val::I32(0)];
        call_instance_function(
            &mut instance,
            "main",
            &[Val::I32(std::env::args().len() as i32), Val::I32(argv_ptr)],
            &mut results,
            &mut store,
        )?;
        results[0].i32().unwrap()
    };

    if main_return_code == 0 {
        Ok(())
    } else {
        anyhow::bail!("main return non zero code")
    }
}
