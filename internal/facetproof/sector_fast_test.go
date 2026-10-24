package facetproof

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

func TestSectorProjectionFloatSeparationIsConservative(t *testing.T) {
	for _, scale := range []float64{1e-120, 1, 1e120} {
		a := []r3.Vec{{Y: 0, Z: 0}, {Y: scale, Z: 0}, {Y: 0, Z: scale}}
		b := []r3.Vec{{Y: 0.9 * scale, Z: 0.9 * scale},
			{Y: scale, Z: 0.9 * scale}, {Y: 0.9 * scale, Z: scale}}
		bound := 0.01 * scale
		aLo, aHi := sectorTestBox(a, bound)
		bLo, bHi := sectorTestBox(b, bound)
		if !sectorPairProjectionSeparateFloat(a, b, bound, bound, aLo, aHi, bLo, bHi) {
			t.Fatalf("float filter did not decide separated sectors at scale %g", scale)
		}

		ny, nz, norm, ok := sectorProjectionAxisFloat(aLo, aHi, bLo, bHi)
		if !ok {
			t.Fatalf("float projection axis failed at scale %g", scale)
		}
		aMin, aMax, ok := sectorProjection(a, bound, proof.FloatRat(ny), proof.FloatRat(nz), norm)
		if !ok {
			t.Fatal("exact projection of the first sector failed")
		}
		bMin, bMax, ok := sectorProjection(b, bound, proof.FloatRat(ny), proof.FloatRat(nz), norm)
		if !ok {
			t.Fatal("exact projection of the second sector failed")
		}
		if aMax.Cmp(bMin) >= 0 && bMax.Cmp(aMin) >= 0 {
			t.Fatalf("float filter separated overlapping exact intervals at scale %g", scale)
		}
		checkFloatEnclosesExact(t, a, bound, ny, nz, norm)
		checkFloatEnclosesExact(t, b, bound, ny, nz, norm)
	}
}

func checkFloatEnclosesExact(t *testing.T, verts []r3.Vec, bound, ny, nz, norm float64) {
	t.Helper()
	flo, fhi, ok := sectorProjectionFloat(verts, ny, nz, norm, bound)
	if !ok {
		t.Fatal("float projection failed")
	}
	exactLo, exactHi, ok := sectorProjection(verts, bound, proof.FloatRat(ny), proof.FloatRat(nz), norm)
	if !ok {
		t.Fatal("exact projection failed")
	}
	if proof.FloatRat(flo).Cmp(exactLo) > 0 || proof.FloatRat(fhi).Cmp(exactHi) < 0 {
		t.Fatal("float projection interval does not enclose the exact interval")
	}
}

func sectorTestBox(verts []r3.Vec, bound float64) (r3.Vec, r3.Vec) {
	lo := r3.Vec{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	hi := r3.Vec{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	for _, p := range verts {
		lo.X, lo.Y, lo.Z = min(lo.X, p.X), min(lo.Y, p.Y), min(lo.Z, p.Z)
		hi.X, hi.Y, hi.Z = max(hi.X, p.X), max(hi.Y, p.Y), max(hi.Z, p.Z)
	}
	lo = r3.Vec{X: math.Nextafter(lo.X-bound, math.Inf(-1)),
		Y: math.Nextafter(lo.Y-bound, math.Inf(-1)), Z: math.Nextafter(lo.Z-bound, math.Inf(-1))}
	hi = r3.Vec{X: math.Nextafter(hi.X+bound, math.Inf(1)),
		Y: math.Nextafter(hi.Y+bound, math.Inf(1)), Z: math.Nextafter(hi.Z+bound, math.Inf(1))}
	return lo, hi
}
