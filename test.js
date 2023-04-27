function f() { print("hi"); }
f();

var x = {};
x.a = 1;
x.b = 2;
x.c = 3;

for (let i = 0; i < 10; i++) {
    print(x.a);
}

var y = {};
y.__proto__ = x;

for (let i = 0; i < 10; i++) {
    print(y.a);
}
