/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * vim: set ts=8 sts=2 et sw=2 tw=80:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/*
 * Whole-program type inference.
 */

#include "vm/WPTI.h"

#include "vm/JSFunction.h"
#include "vm/NativeObject.h"

using namespace js;

bool wpti::Run(JSContext* cx, Handle<GCVector<JSObject*>> roots) {
    // Scan root objects to find JSFunctions.
    Rooted<GCVector<JSFunction*>> funcs(cx, cx);
    for (auto& root : roots) {
        if (root->is<NativeObject>()) {
            Rooted<NativeObject*> nobj(cx, &root->as<NativeObject>());
            for (ShapePropertyIter<NoGC> iter(nobj->shape()); !iter.done(); iter++) {
                if (iter->isDataProperty() && iter->enumerable()) {
                    Value value = nobj->getSlot(iter->slot());
                    if (value.isObject() && value.toObject().is<JSFunction>()) {
                        JSFunction* func = &value.toObject().as<JSFunction>();
                        if (!funcs.append(func)) {
                            return false;
                        }
                    }
                }
            }
        }
    }

    for (auto* func : funcs) {
        Rooted<JSAtom*> explicitName(cx);
        if (func->getExplicitName(cx, &explicitName) && explicitName) {
            UniqueChars chars = StringToNewUTF8CharsZ(cx, *explicitName);
            printf("function: %s\n", chars.get());
        } else {
            printf("function: %p\n", func);
        }
    }
    
    return true;
}
