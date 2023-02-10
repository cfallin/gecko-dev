function main() {
    var accum = 0;
    for (var i = 0; i < 10*1000*1000; i++) {
        accum += i;
    }
    print("accum = " + accum);
}

