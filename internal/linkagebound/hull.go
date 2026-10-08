package linkagebound

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// PrismHull names the recorded section, levels, placement and displacement
// charges used to enclose a straight prism by its outer vertices.
type PrismHull struct {
	Outer                          sectionrecord.LoopRecord
	Frame                          r3.Frame
	Transform                      r3.Transform
	Z0, Z1                         float64
	SectionDelta, Z0Delta, Z1Delta float64
}

// PrismHullPoints returns the outer vertices at both levels as exact rational
// images through the recorded frame and placement. The pad encloses the
// section and level displacements through both coordinate maps. A non-line
// outer segment or an unrepresentable coordinate returns false.
func PrismHullPoints(p PrismHull) ([]motionbound.RatVec, *big.Rat, int, bool) {
	if len(p.Outer.Segments) < 3 {
		return nil, nil, 0, false
	}
	ratOf := func(v r3.Vec) (motionbound.RatVec, bool) { return motionbound.RatVecOf(v) }
	origin, okO := ratOf(p.Frame.Origin())
	fu, okU := ratOf(p.Frame.U())
	fv, okV := ratOf(p.Frame.V())
	fn, okN := ratOf(p.Frame.N())
	basis := p.Transform.Basis()
	ex, okX := ratOf(basis.EX)
	ey, okY := ratOf(basis.EY)
	ez, okZ := ratOf(basis.EZ)
	shift, okT := ratOf(p.Transform.Translation())
	if !okO || !okU || !okV || !okN || !okX || !okY || !okZ || !okT {
		return nil, nil, 0, false
	}
	lift := func(u, v, z *big.Rat) motionbound.RatVec {
		var local, out motionbound.RatVec
		for i := range 3 {
			local[i] = proofbound.RatAdd(origin[i], proofbound.RatMul(fu[i], u), proofbound.RatMul(fv[i], v), proofbound.RatMul(fn[i], z))
		}
		for i := range 3 {
			out[i] = proofbound.RatAdd(proofbound.RatMul(ex[i], local[0]), proofbound.RatMul(ey[i], local[1]),
				proofbound.RatMul(ez[i], local[2]), shift[i])
		}
		return out
	}
	z0, z1 := proofarith.FloatRat(p.Z0), proofarith.FloatRat(p.Z1)
	if z0 == nil || z1 == nil {
		return nil, nil, 0, false
	}
	k := len(p.Outer.Segments)
	points := make([]motionbound.RatVec, 2*k)
	for n, seg := range p.Outer.Segments {
		line, isLine := seg.(sectionrecord.LineSeg)
		if !isLine {
			return nil, nil, 0, false
		}
		u, v := proofarith.FloatRat(line.Start.U), proofarith.FloatRat(line.Start.V)
		if u == nil || v == nil {
			return nil, nil, 0, false
		}
		points[n], points[k+n] = lift(u, v, z0), lift(u, v, z1)
	}
	padF := proofbound.ProductUpper(4, proofbound.Radius3D(max(p.SectionDelta, p.Z0Delta, p.Z1Delta)))
	pad := proofarith.FloatRat(padF)
	if pad == nil {
		return nil, nil, 0, false
	}
	return points, pad, k, true
}

// RestBox returns the exact union of body boxes in one link. Each box's
// reported bound inflates its coordinates before the union is read.
func RestBox(boxes []measurement.Box) (lo, hi motionbound.RatVec, ok bool) {
	for n, box := range boxes {
		bLo, bHi, valid := motionbound.BoxCornersExact(box, new(big.Rat))
		if !valid {
			return motionbound.RatVec{}, motionbound.RatVec{}, false
		}
		if n == 0 {
			lo, hi = bLo, bHi
			continue
		}
		for i := range 3 {
			if bLo[i].Cmp(lo[i]) < 0 {
				lo[i] = bLo[i]
			}
			if bHi[i].Cmp(hi[i]) > 0 {
				hi[i] = bHi[i]
			}
		}
	}
	return lo, hi, true
}
