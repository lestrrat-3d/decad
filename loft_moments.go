package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file is docs/loft-design.md §8's mass-property engine: the exact
// rational tetrahedron-sum accumulator over the assembled wall-and-cap
// triangle set T. Volume and Centroid are polynomial in the vertex
// coordinates, so — following moments.go's own anchor-then-publish-once
// discipline — every vertex coordinate is a float, taken exactly as a
// math/big.Rat, accumulated into one rational sum anchored at the first
// profile's own plane origin, and rounded to float64 exactly ONCE at
// publication. Area has no such closed form (a triangle's own area is a
// square root of a rational, generically irrational) and is never Exact,
// for the same reason spline design §3 gives arc length.

// loftMassAccumulator is docs/loft-design.md §8's tetrahedron-sum kernel. It
// streams one outward-oriented triangle of T at a time — a wall or a cap
// triangulation triangle — anchored at anchor (p0's own PlaneRecord.Origin).
// Volume, Centroid and Bounds fold every triangle handed to add; Area's wall
// contribution folds only the triangles whose wall flag is true, since a
// cap's own contribution is its 2-D region's exact rational area
// (moments.go), never the sum of its triangulation's own float areas.
type loftMassAccumulator struct {
	anchor xpt

	// vol6 is Σ (A-anchor)·((B-anchor)×(C-anchor)) over every triangle of T:
	// six times the signed volume (docs/loft-design.md §8).
	vol6 *big.Rat
	// momX/momY/momZ are Σ vol6_tri · Σ(A-anchor, B-anchor, C-anchor), the
	// first-moment accumulator the centroid divides by 4·vol6.
	momX, momY, momZ *big.Rat

	haveBounds bool
	lo, hi     r3.Vec // componentwise extremes over every held vertex

	wallAreaSum float64 // naive float sum of wall triangles' own areas
	wallTerms   int     // term count sumSlop needs to bound that sum
}

// newLoftMassAccumulator opens a fresh accumulator anchored at the loft's
// first profile plane origin (docs/loft-design.md §8).
func newLoftMassAccumulator(anchor r3.Vec) *loftMassAccumulator {
	return &loftMassAccumulator{
		anchor: xptOf(anchor),
		vol6:   new(big.Rat),
		momX:   new(big.Rat),
		momY:   new(big.Rat),
		momZ:   new(big.Rat),
	}
}

// add folds one outward-oriented triangle (A, B, C) of T into the volume,
// centroid and bounds accumulators, and — when wall is true — into the
// area accumulator's naive float sum. Every vertex coordinate is a float64,
// hence an exact rational (clearance_poly.go's take-the-floats-exactly
// discipline); nothing here rounds until publication.
func (m *loftMassAccumulator) add(a, b, c r3.Vec, wall bool) {
	sa := xsub(xptOf(a), m.anchor)
	sb := xsub(xptOf(b), m.anchor)
	sc := xsub(xptOf(c), m.anchor)

	triVol6 := xdot(sa, xcross(sb, sc))
	m.vol6.Add(m.vol6, triVol6)

	sumX := ratAdd(sa.x, sb.x, sc.x)
	sumY := ratAdd(sa.y, sb.y, sc.y)
	sumZ := ratAdd(sa.z, sb.z, sc.z)
	m.momX.Add(m.momX, new(big.Rat).Mul(triVol6, sumX))
	m.momY.Add(m.momY, new(big.Rat).Mul(triVol6, sumY))
	m.momZ.Add(m.momZ, new(big.Rat).Mul(triVol6, sumZ))

	m.foldBounds(a)
	m.foldBounds(b)
	m.foldBounds(c)

	if !wall {
		return
	}
	u, v := b.Sub(a), c.Sub(a)
	m.wallAreaSum += 0.5 * u.Cross(v).Len()
	m.wallTerms++
}

// foldBounds extends the componentwise extreme box over one held vertex.
// Every vertex is already exact (docs/loft-design.md §5), so comparing them
// introduces no rounding of its own.
func (m *loftMassAccumulator) foldBounds(p r3.Vec) {
	if !m.haveBounds {
		m.lo, m.hi = p, p
		m.haveBounds = true
		return
	}
	m.lo = r3.Vec{X: math.Min(m.lo.X, p.X), Y: math.Min(m.lo.Y, p.Y), Z: math.Min(m.lo.Z, p.Z)}
	m.hi = r3.Vec{X: math.Max(m.hi.X, p.X), Y: math.Max(m.hi.Y, p.Y), Z: math.Max(m.hi.Z, p.Z)}
}

// volume publishes Σvol6/6, rounded to float64 exactly once. Its Exactness
// is exactnessOf the single rounding's proven error — Exact exactly when the
// published rational is representable in cubic millimetres, never
// unconditionally (docs/loft-design.md §8, spline design §3's Tier A rule).
func (m *loftMassAccumulator) volume() Measurement {
	vol := new(big.Rat).Quo(m.vol6, big.NewRat(6, 1))
	value, _ := vol.Float64()
	bound := rationalFloatError(vol, value)
	return Measurement{
		Value:     units.CubicMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.CubicMillimeters(bound),
	}
}

// centroid publishes anchor + Σmoment/(4·Σvol6) as a VecMeasurement, each of
// the three coordinates rounded to float64 exactly once. Bound is radius3D
// of the largest per-coordinate rounding error, Exact only when all three
// round exactly (docs/loft-design.md §8). A loft with zero net volume has no
// centroid.
func (m *loftMassAccumulator) centroid() (VecMeasurement, error) {
	if m.vol6.Sign() == 0 {
		return VecMeasurement{}, fmt.Errorf(`%w: a loft with zero net volume has no centroid`, ErrDegenerate)
	}
	denom := new(big.Rat).Mul(big.NewRat(4, 1), m.vol6)
	cx := new(big.Rat).Add(m.anchor.x, new(big.Rat).Quo(m.momX, denom))
	cy := new(big.Rat).Add(m.anchor.y, new(big.Rat).Quo(m.momY, denom))
	cz := new(big.Rat).Add(m.anchor.z, new(big.Rat).Quo(m.momZ, denom))

	fx, _ := cx.Float64()
	fy, _ := cy.Float64()
	fz, _ := cz.Float64()
	bx := rationalFloatError(cx, fx)
	by := rationalFloatError(cy, fy)
	bz := rationalFloatError(cz, fz)
	bound := radius3D(math.Max(bx, math.Max(by, bz)))

	return VecMeasurement{
		Value:     r3.NewVec(fx, fy, fz),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// bounds publishes the componentwise min/max over every held vertex. It is
// always Exact: every vertex is already exact by construction, so an extreme
// over an already-exact set introduces no rounding of its own
// (docs/loft-design.md §8). The second return is false only when add has
// never been called.
func (m *loftMassAccumulator) bounds() (Box, bool) {
	if !m.haveBounds {
		return Box{}, false
	}
	return Box{
		Min:       m.lo,
		Max:       m.hi,
		Exactness: Exact,
		Bound:     units.Millimeters(0),
	}, true
}

// area publishes the two caps' own exact rational areas (moments.go's
// ProfileRecord.Area, already-exact rationals — never the sum of their own
// triangulations' float areas) plus the wall triangles' naive float sum,
// bounded by sumSlop (the wall sum's own summation-and-square-root proof)
// plus the caps' single-rounding error plus the final addition's own
// rounding. Area is never Exact whenever any wall triangle has nonzero area
// (docs/loft-design.md §8, spline design §3's arc-length asymmetry).
func (m *loftMassAccumulator) area(capAreas ...*big.Rat) Measurement {
	capTotal := new(big.Rat)
	for _, ca := range capAreas {
		if ca != nil {
			capTotal.Add(capTotal, ca)
		}
	}
	capFloat, _ := capTotal.Float64()
	capBound := rationalFloatError(capTotal, capFloat)

	wallBound := sumSlop(m.wallTerms, math.Abs(m.wallAreaSum))
	value := m.wallAreaSum + capFloat
	addBound := addRoundError(m.wallAreaSum, capFloat, value)
	bound := absSumUpper(wallBound, capBound, addBound)

	return Measurement{
		Value:     units.SquareMillimeters(value),
		Exactness: exactnessOf(bound),
		Bound:     units.SquareMillimeters(bound),
	}
}
