open BASE, "<timing.base";

my %base = ();
my @ops = ();

while (<BASE>) {
    if (/op \d+ \(([^\)]*)\): (\d+) count, (\d+) cycles/) {
        my ($name, $count, $cycles) = ($1, $2, $3);
        $base{$name} = [$count, $cycles];
        push @ops, $name;
    }
}

open PBL, "<timing.pbl";

my %pbl = ();

while (<PBL>) {
    if (/op \d+ \(([^\)]*)\): (\d+) count, (\d+) cycles/) {
        my ($name, $count, $cycles) = ($1, $2, $3);
        $pbl{$name} = [$count, $cycles];
    }
}

print "Opcode,Baseline Count,Baseline Cycles,PBL Count,PBL Cycles\n";
for my $op(@ops) {
    my ($base_count, $base_cycles) = @{$base{$op}};
    my ($pbl_count, $pbl_cycles) = @{$pbl{$op}};
    print "$op,$base_count,$base_cycles,$pbl_count,$pbl_cycles\n";
}
