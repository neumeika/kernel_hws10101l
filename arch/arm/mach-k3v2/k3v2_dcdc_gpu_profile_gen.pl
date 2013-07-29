#!/usr/bin/perl -w

use Spreadsheet::ParseExcel;
use strict;

my $parser = new Spreadsheet::ParseExcel;
my $Book = $parser->Parse($ARGV[0]) ||
	die("Failed to open file $ARGV[0]\n");

if ( !defined $Book ) {
    die $parser->error(), ".\n";
}

open(OUTPUT, ">$ARGV[1]") ||
	die("Failed to open file $ARGV[1]\n");

($_ = $0) =~ s:.*/::;

print OUTPUT "\n/* DO NOT EDIT - Generated automatically by ".$_." */\n\n\n";
print OUTPUT "#include \"k3v2_dcdc_gpu.h\"\n\n";


#GPU
#------------------------------------------------------------------------------
my $Sheet = $Book->{Worksheet}[2];
print OUTPUT "struct gpu_profile_info gpu_profile[] = {\n";
print OUTPUT "\t/*freq, gpu_clk_profile, gpu_vol_profile, dcdc_vol*/\n";
foreach my $col ( 0 .. 7) {
	if($Sheet->{Cells}[$col+1][2]->Value eq '') {
		last;
	}
	printf OUTPUT "\t{%d, 0x%s, 0x0000%s, %d000},\n",
		($Sheet->{Cells}[$col+1][1]->Value)*1000,
		($Sheet->{Cells}[$col+1][33]->Value),
		substr($Sheet->{Cells}[$col+1][31]->Value, 4, 4),
		($Sheet->{Cells}[$col+1][5]->Value)*1000;
}
print OUTPUT "\t{0, 0, 0, 0},\n};\n";

foreach my $row ( 0 .. 13) {
	if($row eq 0) {
		print OUTPUT "struct gpu_policy_info gpu_policy_powersave[] = {\n";
	} elsif($row eq 1) {
		print OUTPUT "struct gpu_policy_info gpu_policy_normal[] = {\n";
	} elsif($row eq 2) {
		print OUTPUT "struct gpu_policy_info gpu_policy_performance[] = {\n";
	} else {
		printf OUTPUT "struct gpu_policy_info gpu_policy_special%02X[] = {\n", ($row-2);
	}
	print OUTPUT "\t/*uptimes, downtimes, up_threshold, down_threshold*/\n";
	foreach my $col ( 0 .. 7) {
		if($Sheet->{Cells}[$col+1][2]->Value eq '') {
			last;
		}
		printf OUTPUT "\t{%d, %d, %d, %d},\n",
			($Sheet->{Cells}[$row*15+$col+16][4]->Value)/($Sheet->{Cells}[14][3]->Value),
			($Sheet->{Cells}[$row*15+$col+16][7]->Value)/($Sheet->{Cells}[14][3]->Value),
			substr($Sheet->{Cells}[$row*15+$col+16][2]->Value,0,index($Sheet->{Cells}[$row*15+$col+16][2]->Value,".")),
			substr($Sheet->{Cells}[$row*15+$col+16][5]->Value,0,index($Sheet->{Cells}[$row*15+$col+16][5]->Value,"."));
	}
	print OUTPUT "\t{0, 0, 0, 0},\n};\n";
}

print OUTPUT "struct gpu_policy_info *policy_table[] = {\n";
foreach my $row (0 .. 13) {
	if($row eq 0) {
		print OUTPUT "\t[NORMAL_POLICY] = gpu_policy_normal,\n";
	} elsif($row eq 1) {
		print OUTPUT "\t[POWERSAVE_POLICY] = gpu_policy_powersave,\n";
	} elsif($row eq 2) {
		print OUTPUT "\t[PERF_POLICY] = gpu_policy_performance,\n";
	} else {
		printf OUTPUT "\t[SPEC%02X_POLICY] = gpu_policy_special%02X,\n", ($row-2), ($row-2);
	}
}
print OUTPUT "};\n";

close OUTPUT;
