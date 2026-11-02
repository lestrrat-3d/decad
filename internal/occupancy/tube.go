package occupancy

import (
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Arc tube leaves (agent-docs/body-refine-design.md §4): the solid a disc of
// radius rho sweeps when its centre runs along the circle o + R·(e0·cos φ +
// e1·sin φ), φ in [phi0, phi1], with the disc in the plane through the circle's
// axis w — a revolved circle profile, or one arc span of a swept one.
//
// These leaves are read in floats, not exact rationals: the spine points and
// the frames of the outer pieces come from float trigonometry. Every reading
// is widened by relSlack of its own magnitude plus the caller's slack, which
// covers float evaluation many orders of magnitude over; it is a charged
// allowance, not an exact proof, and the design says so.
const relSlack = 1e-12

// maxArcCells bounds the outer pieces one arc leaf builds.
const maxArcCells = 4096

// ArcTubeLeaf builds the leaf of one arc tube. Each angular cell's piece is a
// circumscribed prism about the chord between its two spine points, wide
// enough for the arc's own sagitta over the cell and long enough for the disc
// at either end to tilt by half the cell's angle. cellSagitta caps that
// sagitta and so fixes the cell count. delta is the published bound of the
// held mesh the Boolean used.
func ArcTubeLeaf(o, w, e0, e1 r3.Vec, radius, rho, phi0, phi1 float64, full bool,
	slack, cellSagitta float64, sides int, delta float64) (Leaf, error) {
	if slices.ContainsFunc([]float64{radius, rho, phi0, phi1, slack, cellSagitta, delta}, proofbound.IsNonFinite) {
		return Leaf{}, fmt.Errorf("%w: arc tube reading", ErrInvalid)
	}
	if full {
		phi0, phi1 = 0, 2*math.Pi
	}
	if phi1 < phi0 {
		phi0, phi1 = phi1, phi0
	}
	span := phi1 - phi0
	if !(radius > rho) || !(rho > 0) || !(span > 0) || span > 2*math.Pi+1e-12 || !(cellSagitta > 0) || slack < 0 || delta < 0 {
		return Leaf{}, fmt.Errorf("%w: arc tube of radius %v, tube %v, span %v", ErrInvalid, radius, rho, span)
	}
	// Cell angle d with radius·d²/8 ≤ cellSagitta.
	cells := int(math.Ceil(span / math.Sqrt(8*cellSagitta/radius)))
	cells = max(cells, 4)
	if cells > maxArcCells {
		return Leaf{}, fmt.Errorf("%w: an arc tube asks for %d outer pieces", ErrInvalid, cells)
	}
	scale := o.Len() + radius + rho
	pad := proofbound.AbsSumUpper(slack, relSlack*scale)
	d := span / float64(cells)
	sag := radius * d * d / 8
	ext := proofbound.AbsSumUpper(rho*d/2, pad)
	tubeR := proofbound.AbsSumUpper(rho, sag, pad)
	spine := func(phi float64) r3.Vec {
		s, c := math.Sincos(phi)
		return o.Add(e0.Scale(radius * c)).Add(e1.Scale(radius * s))
	}
	boxes := make([][2]r3.Vec, cells)
	for k := range cells {
		pa, pb := spine(phi0+float64(k)*d), spine(phi0+float64(k+1)*d)
		// The piece reaches tubeR/cos(π/sides) from its chord, and ext past
		// either end.
		boxes[k] = cellBox(pa, pb, tubeR/math.Cos(math.Pi/float64(sides))*(1+1e-9)+ext)
	}
	build := func() ([]*Piece, error) {
		pieces := make([]*Piece, 0, cells)
		for k := range cells {
			pa, pb := spine(phi0+float64(k)*d), spine(phi0+float64(k+1)*d)
			piece, err := SegmentTube(pa, pb, w, tubeR, ext, sides)
			if err != nil {
				return nil, err
			}
			pieces = append(pieces, piece)
		}
		return pieces, nil
	}
	sh := torusShape{o: o, w: w, e0: e0, e1: e1, radius: radius, rho: rho, phi0: phi0, phi1: phi1,
		full: full || span >= 2*math.Pi, pad: pad}
	l := newLeaf(boxes, build, delta, sh)
	l.key = fmt.Sprintf("arc %v %v %v %v %v %v %v %v %v %v %v", o, w, e0, e1, radius, rho, phi0, phi1, full, slack, cellSagitta)
	return l, nil
}

// SegmentTube is the circumscribed prism of radius r about the segment a→b,
// extended by ext past both ends. hint is any direction not along the
// segment; it fixes the prism's roll.
func SegmentTube(a, b, hint r3.Vec, r, ext float64, sides int) (*Piece, error) {
	axis := b.Sub(a)
	l := axis.Len()
	d, ok := axis.Normalize()
	if !ok {
		return nil, fmt.Errorf("%w: a segment tube needs two distinct ends", ErrInvalid)
	}
	u, ok := hint.Sub(d.Scale(hint.Dot(d))).Normalize()
	if !ok {
		if u, ok = d.Cross(r3.Vec{X: 1}).Normalize(); !ok {
			u, _ = d.Cross(r3.Vec{Y: 1}).Normalize()
		}
	}
	v := d.Cross(u)
	// The float frame is orthonormal to rounding, so its exact image of the
	// local prism contains the Euclidean one of a radius smaller by far less
	// than relSlack of the piece's own size, which r carries.
	m := Affine{M: [3][3]float64{{u.X, v.X, d.X}, {u.Y, v.Y, d.Y}, {u.Z, v.Z, d.Z}}, T: [3]float64{a.X, a.Y, a.Z}}
	pad := relSlack * (a.Len() + l + r)
	return CirclePrism(Chain{m}, r+pad, -ext-pad, l+ext+pad, sides)
}

// torusShape reads how far a held triangle pokes out of an arc tube. Write a
// point's meridian offset from the spine as P = (r − R, h), with r its
// distance from the axis and h its height along it. h is affine over the
// triangle. r is convex, and over a triangle whose corners span an angle α
// about the axis it lies between its corner interpolation r_lin and
// r_lin·cos(α/2) (take the bisector of the corners' directions). So
// P = (A − c, h) with A = r_lin − R affine and 0 ≤ c ≤ r_max·(1 − cos(α/2)).
// Then |P|² = |(A, h)|² − 2·A·c + c² ≤ M² + 2·N·c + c², where M is the largest
// |(A, h)| (convex, so at a corner) and N the largest −A (affine, so at a
// corner), floored at zero: a facet on the tube's outer side, where A ≥ 0,
// pokes out by at most c²/(2ρ), and one on its inner side by about c. A corner
// outside the arc's angular range by more than a hair, or corners spanning
// half a turn, answer +Inf.
type torusShape struct {
	o, w, e0, e1 r3.Vec
	radius, rho  float64
	phi0, phi1   float64
	full         bool
	pad          float64
}

func (s torusShape) poke(tri [3]r3.Vec) float64 {
	var dir [3][2]float64
	corner, rMax, inner, angle := 0.0, 0.0, 0.0, 0.0
	for i, x := range tri {
		rel := x.Sub(s.o)
		h := rel.Dot(s.w)
		a, b := rel.Dot(s.e0), rel.Dot(s.e1)
		r := math.Hypot(a, b)
		if !(r > 0) {
			return math.Inf(1)
		}
		dir[i] = [2]float64{a / r, b / r}
		corner = math.Max(corner, math.Hypot(r-s.radius, h))
		rMax = math.Max(rMax, r)
		inner = math.Max(inner, s.radius-r)
		if !s.full {
			mid := (s.phi0 + s.phi1) / 2
			off := math.Abs(math.Remainder(math.Atan2(b, a)-mid, 2*math.Pi))
			angle = math.Max(angle, off-(s.phi1-s.phi0)/2)
		}
	}
	if s.phi1-s.phi0 > math.Pi+1e-12 && !s.full {
		return math.Inf(1)
	}
	if angle > 1e-6 {
		return math.Inf(1)
	}
	// The corners' angular span: the largest pairwise angle, read through the
	// cosine of each pair; a span of half a turn or more has no bisector.
	cosHalf := 1.0
	for i := range 3 {
		for k := i + 1; k < 3; k++ {
			c := dir[i][0]*dir[k][0] + dir[i][1]*dir[k][1]
			if c <= -1+1e-9 {
				return math.Inf(1)
			}
			// cos(α/2) = sqrt((1 + cos α)/2).
			cosHalf = math.Min(cosHalf, math.Sqrt(math.Max(0, (1+c)/2)))
		}
	}
	defect := rMax * (1 - cosHalf) * (1 + relSlack)
	out := math.Sqrt(corner*corner+2*inner*defect+defect*defect)*(1+relSlack) - s.rho
	if angle > 0 {
		out += 2 * (s.radius + s.rho) * angle
	}
	return proofbound.AbsSumUpper(math.Max(out*(1+relSlack), 0), s.pad)
}

// boundaryFar is not read for arc tubes: no Cut takes one as a tool.
func (torusShape) boundaryFar(r3.Vec, r3.Vec, float64) bool { return false }

// HelixTubeLeaf builds the leaf of a Coil of a whole circle: the disc of radius
// rho centred at axis coordinates (rc, zc) moved by the screw motion
// Φ(ρ, ζ, θ) = c + ρ·(cos θ·er + σ·sin θ·et) + (ζ + k·θ)·n for θ in [0, span]
// (agent-docs/helix-design.md Table CP). Every point of the solid lies within
// rho of the centre helix at its own θ, since the disc sits in the axis plane
// there; each angular cell's piece is a circumscribed prism about the chord of
// the centre helix over the cell, widened by the helix's sagitta over it
// (curvature rc/(rc² + k²) over an arc of length d·sqrt(rc² + k²) gives
// rc·d²/8) and extended past both ends by how far a disc offset can reach
// along the chord. The leaf reads no poke: every held facet keeps its own
// bound as its bulge. delta is the published bound of the held mesh.
func HelixTubeLeaf(c, n, er, et r3.Vec, sigma, k, rc, zc, rho, span, slack, cellSagitta float64, sides int, delta float64) (Leaf, error) {
	if slices.ContainsFunc([]float64{sigma, k, rc, zc, rho, span, slack, cellSagitta, delta}, proofbound.IsNonFinite) {
		return Leaf{}, fmt.Errorf("%w: helix tube reading", ErrInvalid)
	}
	if !(rc > rho) || !(rho > 0) || !(span > 0) || !(cellSagitta > 0) || slack < 0 || delta < 0 {
		return Leaf{}, fmt.Errorf("%w: helix tube of radius %v, wire %v, span %v", ErrInvalid, rc, rho, span)
	}
	cells := max(int(math.Ceil(span/math.Sqrt(8*cellSagitta/rc))), 4)
	if cells > 4*maxArcCells {
		return Leaf{}, fmt.Errorf("%w: a helix tube asks for %d outer pieces", ErrInvalid, cells)
	}
	scale := c.Len() + rc + rho + math.Abs(zc) + math.Abs(k)*span
	pad := proofbound.AbsSumUpper(slack, relSlack*scale)
	d := span / float64(cells)
	sag := rc * d * d / 8
	lead := math.Abs(k) / math.Hypot(rc, k)
	ext := proofbound.AbsSumUpper(rho*(lead+d), pad)
	tubeR := proofbound.AbsSumUpper(rho, sag, pad)
	helix := func(theta float64) r3.Vec {
		s, co := math.Sincos(theta)
		return c.Add(er.Scale(rc * co)).Add(et.Scale(sigma * rc * s)).Add(n.Scale(zc + k*theta))
	}
	reach := tubeR/math.Cos(math.Pi/float64(sides))*(1+1e-9) + ext
	boxes := make([][2]r3.Vec, cells)
	for i := range cells {
		boxes[i] = cellBox(helix(float64(i)*d), helix(float64(i+1)*d), reach)
	}
	build := func() ([]*Piece, error) {
		pieces := make([]*Piece, 0, cells)
		for i := range cells {
			theta := (float64(i) + 0.5) * d
			s, co := math.Sincos(theta)
			radial := er.Scale(co).Add(et.Scale(sigma * s))
			piece, err := SegmentTube(helix(float64(i)*d), helix(float64(i+1)*d), radial, tubeR, ext, sides)
			if err != nil {
				return nil, err
			}
			pieces = append(pieces, piece)
		}
		return pieces, nil
	}
	l := newLeaf(boxes, build, delta, noPoke{})
	l.key = fmt.Sprintf("helix %v %v %v %v %v %v %v %v %v %v %v %v", c, n, er, et, sigma, k, rc, zc, rho, span, slack, cellSagitta)
	return l, nil
}

// noPoke is the shape of a leaf that bounds no facet's poke: every facet
// falls back to its own two-sided bound.
type noPoke struct{}

func (noPoke) poke([3]r3.Vec) float64                   { return math.Inf(1) }
func (noPoke) boundaryFar(r3.Vec, r3.Vec, float64) bool { return false }
