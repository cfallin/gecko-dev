function f() {
    try {
        g(42);
    } catch (e) {
        print("caught: " + e);
    } finally {
        print("finally");
    }
}

function g(value) {
    throw value;
}

function main() {
    f();
}

