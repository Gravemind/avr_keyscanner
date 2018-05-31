#!/usr/bin/perl
#

use warnings;
use strict;
use IPC::Open2;
use File::Basename;

my @debouncers = ();

if (@ARGV) {
    while (@ARGV) {
        push @debouncers, shift @ARGV;
    }
}
else {
    for my $debouncer (`find . -path './debounce-*' -type f -executable`) {
        chomp $debouncer;

        #next if $debouncer eq './debounce-none';
        #next if $debouncer eq './debounce-counter';
        #next if $debouncer eq './debounce-split-counters-and-lockouts';
        push @debouncers, $debouncer;
    }
}

my @testcases;
for my $test (`find ../testcases -type f`) {
    chomp $test;
    next if ( $test =~ /\.(raw|bak|log\.txt)$/ );
    push @testcases, $test;
}

@debouncers = sort @debouncers;
@testcases  = sort @testcases;

my %stats_by_db;
my %stats_by_test;

my $args = '';
for my $test (@testcases) {
    $args .= ' ';
    $args .= $test;
}

for my $debouncer (@debouncers) {

    print STDERR "Running ", $debouncer, "...\n";

    my $pid = open2( \*CHLD_OUT, \*CHLD_IN, $debouncer . " " . $args )
      or die "open2() failed $!";
    close(CHLD_IN);

    while ( my $line = <CHLD_OUT> ) {

        chomp($line);
        my @column = split( ';', $line );

        #my $res_db = $column[0];
        my $res_db   = $debouncer;
        my $res_test = $column[1];
        my $res_res  = $column[2];

        #$stats_by_db{$res_db}{$res_test} = $res_res;
        $stats_by_test{$res_test}{$res_db} .= $res_res;
    }

    close(CHLD_OUT);

    waitpid( $pid, 0 );
    my $child_exit_status = $?;
    if ( $child_exit_status != 0 ) {
        print STDERR "\nFAIL: ", $debouncer, " exited with failure ", $child_exit_status, " !\n\n";
    }
}

my $html_head = <<'END_MESSAGE';
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8" />

<link rel="stylesheet" type="text/css" href="https://cdn.datatables.net/1.10.16/css/jquery.dataTables.min.css">
<script type="text/javascript" src="https://code.jquery.com/jquery-1.12.4.js"></script>
<script type="text/javascript" src="https://cdn.datatables.net/1.10.16/js/jquery.dataTables.min.js"></script>

<!--
<link rel="stylesheet" type="text/css" href="jquery.dataTables.min.css">
<script type="text/javascript" src="jquery-1.12.4.js"></script>
<script type="text/javascript" src="jquery.dataTables.min.js"></script>
-->

<script type="text/javascript" src="test_results.js"></script>

<style>
body { font-size: 13px; font-family: sans-serif; }
</style>

</head>
<body>
END_MESSAGE

my $html_tail = <<'END_MESSAGE';
</body></html>
END_MESSAGE

printf $html_head;

printf '<form>';
printf '<a href="" id="vis-all">All</a> ';
printf '<a href="" id="vis-none">None</a> ';
printf '<br/>';
my $i = 1;
for my $debouncer (@debouncers) {
    my $db_name = $debouncer;
    $db_name =~ s/^.*debounce-//;
    printf
'<input class="toggle-vis" type="checkbox" id="viscol%d" data-column="%d" checked/><label for="viscol%d">%s</label> ',
      $i, $i, $i, $db_name;
    $i++;
}
printf '</form>';

printf
  '<table id="table" class="display compact"><thead><tr><th>test/deboucer</th>';
for my $debouncer (@debouncers) {
    my $db_name = $debouncer;
    $db_name =~ s/^.*debounce-//;
    printf '<th>%s</th>', $db_name;
}
printf "</tr></thead><tbody>\n";

my @tests = keys %stats_by_test;
for my $test (@tests) {
    my $test_name = $test;

    #$test_name =~ s/^.*testcases\/(.*?)(?:\.data)?$/$1/;
    #$test_name =~ s/^.*testcases\/(.*?)(?:\.data)?$/$1/;
    printf "<tr><th>%s</th>", $test_name;
    for my $debouncer (@debouncers) {
        if ( exists $stats_by_test{$test}{$debouncer} ) {
            printf '<td>%s</td>', $stats_by_test{$test}{$debouncer};
        }
        else {
            printf "<td></td>";
        }
    }
    printf "</tr>\n";
}
printf "</tbody></table>\n";
printf $html_tail;
