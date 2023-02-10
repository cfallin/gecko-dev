function fib(n) {
    if (n < 2) {
        return 1;
    } else {
        return fib(n-1) + fib(n-2);
    }
}

function main() {
    print(fib(30));
}
