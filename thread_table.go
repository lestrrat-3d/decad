package decad

import (
	"fmt"
	"math"
	"strconv"
	"strings"

	"github.com/lestrrat-3d/units"
)

// ThreadSize is a standard thread's major diameter and pitch, as [LookupThread]
// reads them from its designation.
type ThreadSize struct {
	// Major is the thread's nominal (outside) diameter and Pitch its axial
	// advance per turn, both lengths.
	Major, Pitch units.Value
}

// metricCoarse is the ISO 261 coarse pitch in millimetres of each preferred
// nominal diameter of the ISO metric series, M1 to M64.
var metricCoarse = map[float64]float64{
	1: 0.25, 1.2: 0.25, 1.4: 0.3, 1.6: 0.35, 1.8: 0.35, 2: 0.4, 2.5: 0.45, 3: 0.5, 3.5: 0.6, 4: 0.7,
	5: 0.8, 6: 1, 8: 1.25, 10: 1.5, 12: 1.75, 14: 2, 16: 2, 18: 2.5, 20: 2.5, 22: 2.5, 24: 3, 27: 3,
	30: 3.5, 33: 3.5, 36: 4, 39: 4, 42: 4.5, 45: 4.5, 48: 5, 52: 5, 56: 5.5, 60: 5.5, 64: 6,
}

// unified holds the threads per inch of the Unified series by nominal diameter in
// sixty-fourths of an inch, coarse (UNC) then fine (UNF), for the fractional sizes
// from 1/4 inch to 1 inch.
var unified = map[int][2]float64{
	16: {20, 28}, 20: {18, 24}, 24: {16, 24}, 28: {14, 20}, 32: {13, 20}, 36: {12, 18},
	40: {11, 18}, 48: {10, 16}, 56: {9, 14}, 64: {8, 12},
}

// numbered holds the threads per inch of the Unified numbered sizes #0 to #12 by number, coarse (UNC) then fine
// (UNF), zero where the size has none: #0 has only a fine thread. The nominal diameter of size #n is
// 0.060 + 0.013·n inch.
var numbered = map[int][2]float64{
	0: {0, 80}, 1: {64, 72}, 2: {56, 64}, 3: {48, 56}, 4: {40, 48}, 5: {40, 44}, 6: {32, 40}, 8: {32, 36},
	10: {24, 32}, 12: {24, 28},
}

// pipeG holds the major diameter in millimetres and the threads per inch of the ISO 228 parallel pipe threads (BSPP,
// G) by size designation, the size being the nominal bore in inches as the standard writes it.
var pipeG = map[string][2]float64{
	"1/8": {9.728, 28}, "1/4": {13.157, 19}, "3/8": {16.662, 19}, "1/2": {20.955, 14}, "5/8": {22.911, 14},
	"3/4": {26.441, 14}, "7/8": {30.201, 14}, "1": {33.249, 11}, "1-1/4": {41.910, 11}, "1-1/2": {47.803, 11},
	"2": {59.614, 11},
}

// LookupThread reads a thread designation into its major diameter and pitch
// (agent-docs/thread-design.md §9). It reads five forms:
//
//   - "M10", an ISO metric thread at its coarse pitch: 10 mm and 1.5 mm. The
//     nominal diameter is one of the preferred sizes from M1 to M64.
//   - "M10x1.25", a metric thread with the pitch stated, any positive pitch
//     below the diameter.
//   - "1/4-20" and "1/4-28", and "1-8", a Unified thread by nominal fraction and
//     threads per inch: 6.35 mm and 25.4/20 mm. The size is a multiple of 1/16 inch
//     from 1/4 to 1 inch, and the threads per inch are its UNC or UNF value.
//   - "#10-24" and "#10-32", a Unified numbered size by its number and threads per
//     inch: 0.190 inch and 25.4/24 mm. The size is #0 to #6, #8, #10 or #12, and the
//     threads per inch are its UNC or UNF value.
//   - "G1/2", an ISO 228 parallel pipe thread by its nominal bore: 20.955 mm and 25.4/14 mm. The size is one of
//     G1/8, G1/4, G3/8, G1/2, G5/8, G3/4, G7/8, G1, G1-1/4, G1-1/2 and G2. The size gives the diameter and pitch only:
//     the thread's 55 degree Whitworth profile is not cut, and the caller picks the form.
//
// The designation is case-insensitive in its M and x and takes no spaces. The
// pitch is the thread's own and the major diameter is its nominal one: tolerance
// classes, allowances and a thread's minor diameter are not read. A designation
// that is none of these, or whose size is not in the tables, is ErrDegenerate.
func LookupThread(designation string) (ThreadSize, error) {
	d := strings.TrimSpace(designation)
	if d == "" {
		return ThreadSize{}, fmt.Errorf(`%w: a thread designation is empty`, ErrDegenerate)
	}
	if d[0] == 'M' || d[0] == 'm' {
		return lookupMetric(d)
	}
	if d[0] == 'G' || d[0] == 'g' {
		return lookupPipe(d)
	}
	if d[0] == '#' {
		return lookupNumbered(d)
	}
	return lookupUnified(d)
}

func lookupPipe(d string) (ThreadSize, error) {
	rates, ok := pipeG[d[1:]]
	if !ok {
		return ThreadSize{}, fmt.Errorf(`%w: %q is not an ISO 228 pipe thread size such as G1/2`, ErrDegenerate, d)
	}
	return ThreadSize{Major: units.Millimeters(rates[0]), Pitch: units.Inches(1 / rates[1])}, nil
}

func lookupNumbered(d string) (ThreadSize, error) {
	number, tpiText, ok := strings.Cut(d[1:], "-")
	if !ok {
		return ThreadSize{}, fmt.Errorf(`%w: %q is not a numbered Unified designation like #10-24`, ErrDegenerate, d)
	}
	n, errN := strconv.Atoi(number)
	tpi, errT := strconv.ParseFloat(tpiText, 64)
	if errN != nil || errT != nil || n < 0 || !(tpi > 0) || math.IsInf(tpi, 0) {
		return ThreadSize{}, fmt.Errorf(`%w: %q is not a numbered Unified designation like #10-24`, ErrDegenerate, d)
	}
	rates, listed := numbered[n]
	if !listed || (tpi != rates[0] && tpi != rates[1]) {
		return ThreadSize{}, fmt.Errorf(`%w: %q is not a coarse or fine Unified size from #0 to #12`, ErrDegenerate, d)
	}
	return ThreadSize{Major: units.Inches(0.060 + 0.013*float64(n)), Pitch: units.Inches(1 / tpi)}, nil
}

func lookupMetric(d string) (ThreadSize, error) {
	body := d[1:]
	nominal, pitchText, fine := strings.Cut(strings.ToLower(body), "x")
	major, err := strconv.ParseFloat(nominal, 64)
	if err != nil || !(major > 0) || math.IsInf(major, 0) {
		return ThreadSize{}, fmt.Errorf(`%w: %q is not a metric thread designation like M10 or M10x1.25`, ErrDegenerate, d)
	}
	pitch := 0.0
	if fine {
		pitch, err = strconv.ParseFloat(pitchText, 64)
		if err != nil || !(pitch > 0) || math.IsInf(pitch, 0) {
			return ThreadSize{}, fmt.Errorf(`%w: %q has no readable pitch after the x`, ErrDegenerate, d)
		}
		if pitch >= major {
			return ThreadSize{}, fmt.Errorf(`%w: the pitch %g mm of %q is not under its diameter`, ErrDegenerate, pitch, d)
		}
	} else {
		var ok bool
		if pitch, ok = metricCoarse[major]; !ok {
			return ThreadSize{}, fmt.Errorf(`%w: M%s is not one of the preferred metric sizes M1 to M64; give the pitch as in M%sx1.5`,
				ErrDegenerate, nominal, nominal)
		}
	}
	return ThreadSize{Major: units.Millimeters(major), Pitch: units.Millimeters(pitch)}, nil
}

func lookupUnified(d string) (ThreadSize, error) {
	fraction, tpiText, ok := strings.Cut(d, "-")
	if !ok {
		return ThreadSize{}, fmt.Errorf(`%w: %q is not a thread designation like M10 or 1/4-20`, ErrDegenerate, d)
	}
	// A whole number of inches, as in 1-8, is that number over one.
	num, den, slash := strings.Cut(fraction, "/")
	if !slash {
		num, den = fraction, "1"
	}
	n, errN := strconv.Atoi(num)
	m, errM := strconv.Atoi(den)
	tpi, errT := strconv.ParseFloat(tpiText, 64)
	if errN != nil || errM != nil || errT != nil || n <= 0 || m <= 0 || !(tpi > 0) || math.IsInf(tpi, 0) {
		return ThreadSize{}, fmt.Errorf(`%w: %q is not a Unified thread designation like 1/4-20`, ErrDegenerate, d)
	}
	sixtyFourths := n * 64 / m
	if n*64%m != 0 {
		return ThreadSize{}, fmt.Errorf(`%w: %q is not a multiple of 1/64 inch`, ErrDegenerate, d)
	}
	rates, listed := unified[sixtyFourths]
	if !listed || (tpi != rates[0] && tpi != rates[1]) {
		return ThreadSize{}, fmt.Errorf(`%w: %q is not a coarse or fine Unified size from 1/4 to 1 inch`, ErrDegenerate, d)
	}
	return ThreadSize{Major: units.Inches(float64(sixtyFourths) / 64), Pitch: units.Inches(1 / tpi)}, nil
}
