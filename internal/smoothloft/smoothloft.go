package smoothloft

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// Build evaluates docs/loft-sections-design.md's exact positive homothetic
// class. It returns a closed held mesh and non-cancelling certificates for the
// quadratic surface which that mesh represents.
type Build struct {
	Vertices   []r3.Vec
	Triangles  [][3]int
	Sources    []int
	EdgeCount  int
	MeshBound  float64
	VolSymDiff float64
	AreaSlack  float64
	Diameter   float64
}

func rint(n int64) *big.Rat      { return big.NewRat(n, 1) }
func rf(f float64) *big.Rat      { return new(big.Rat).SetFloat64(f) }
func add(x, y *big.Rat) *big.Rat { return new(big.Rat).Add(x, y) }
func sub(x, y *big.Rat) *big.Rat { return new(big.Rat).Sub(x, y) }
func mul(x, y *big.Rat) *big.Rat { return new(big.Rat).Mul(x, y) }
func div(x, y *big.Rat) *big.Rat { return new(big.Rat).Quo(x, y) }
func abs(x *big.Rat) *big.Rat    { return new(big.Rat).Abs(x) }

func maxRat(x, y *big.Rat) *big.Rat {
	if x.Cmp(y) >= 0 {
		return x
	}
	return y
}

func exactFloat(q *big.Rat) (float64, error) {
	f, _ := q.Float64()
	if math.IsNaN(f) || math.IsInf(f, 0) || rf(f).Cmp(q) != 0 {
		return 0, fmt.Errorf("%w: a three-section loft station cannot be held exactly in float64", decaderr.ErrUnsupported)
	}
	return f, nil
}

func walkedPoints(profile momentinput.Profile) ([]sectionrecord.Point2, error) {
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) < 3 || len(profile.Outer.Segments) > 64 {
		return nil, fmt.Errorf("%w: a three-section loft needs one whole-line outer loop of 3 to 64 edges", decaderr.ErrUnsupported)
	}
	points := make([]sectionrecord.Point2, len(profile.Outer.Segments))
	ends := make([]sectionrecord.Point2, len(points))
	for i, segment := range profile.Outer.Segments {
		line, ok := segment.(sectionrecord.LineSeg)
		if !ok {
			return nil, fmt.Errorf("%w: a three-section loft needs whole line segments", decaderr.ErrUnsupported)
		}
		switch {
		case line.TStart == 0 && line.TEnd == 1:
			points[i], ends[i] = line.Start, line.End
		case line.TStart == 1 && line.TEnd == 0:
			points[i], ends[i] = line.End, line.Start
		default:
			return nil, fmt.Errorf("%w: a three-section loft cannot use a trimmed line", decaderr.ErrUnsupported)
		}
	}
	for i := range points {
		if ends[i] != points[(i+1)%len(points)] {
			return nil, fmt.Errorf("%w: a three-section loft line loop has unequal recorded joins", decaderr.ErrUnsupported)
		}
	}
	return points, nil
}

func areaOf(points []sectionrecord.Point2) *big.Rat {
	twice := new(big.Rat)
	for i, p := range points {
		q := points[(i+1)%len(points)]
		twice = add(twice, sub(mul(rf(p.U), rf(q.V)), mul(rf(p.V), rf(q.U))))
	}
	return div(twice, rint(2))
}

// originInKernel proves every radial segment from the common scale origin
// stays inside the first CCW profile. This exact half-plane test makes all
// positive-scaled sections nested, including concave ones.
func originInKernel(points []sectionrecord.Point2) bool {
	for i, p := range points {
		q := points[(i+1)%len(points)]
		dx, dy := sub(rf(q.U), rf(p.U)), sub(rf(q.V), rf(p.V))
		cross := sub(mul(dy, rf(p.U)), mul(dx, rf(p.V)))
		if cross.Sign() < 0 {
			return false
		}
	}
	return true
}

// scaleOf compares every coordinate in exact rational arithmetic. The first
// nonzero base coordinate fixes the sole candidate scale; no residual admits
// a different correspondence.
func scaleOf(base, other []sectionrecord.Point2) (*big.Rat, error) {
	if len(base) != len(other) {
		return nil, fmt.Errorf("%w: three-section loft profiles have different edge counts", decaderr.ErrUnsupported)
	}
	var scale *big.Rat
	for i, p := range base {
		for axis, v := range [...]float64{p.U, p.V} {
			otherV := other[i].U
			if axis == 1 {
				otherV = other[i].V
			}
			if v == 0 {
				if otherV != 0 {
					return nil, fmt.Errorf("%w: three-section loft profiles are not homothetic", decaderr.ErrUnsupported)
				}
				continue
			}
			if scale == nil {
				scale = div(rf(otherV), rf(v))
			}
			if mul(scale, rf(v)).Cmp(rf(otherV)) != 0 {
				return nil, fmt.Errorf("%w: three-section loft profiles are not homothetic", decaderr.ErrUnsupported)
			}
		}
	}
	if scale == nil || scale.Sign() <= 0 {
		return nil, fmt.Errorf("%w: three-section loft scale must be positive", decaderr.ErrUnsupported)
	}
	return scale, nil
}

func quadraticAt(a, b, t *big.Rat) *big.Rat {
	return add(rint(1), add(mul(b, t), mul(a, mul(t, t))))
}

func squaredIntegral(a, b *big.Rat) *big.Rat {
	coeff := [...]*big.Rat{rint(1), b, a}
	sum := new(big.Rat)
	for i, x := range coeff {
		for j, y := range coeff {
			sum = add(sum, div(mul(x, y), rint(int64(i+j+1))))
		}
	}
	return sum
}

func areaSlack(points []sectionrecord.Point2, a, b, qMax, h *big.Rat, n int) *big.Rat {
	nn := rint(int64(n))
	d := div(abs(a), mul(rint(4), mul(nn, nn)))
	dPrime := div(abs(a), nn)
	m := maxRat(abs(b), abs(add(mul(rint(2), a), b)))
	sum := new(big.Rat)
	for i, p := range points {
		q := points[(i+1)%len(points)]
		dx, dy := sub(rf(q.U), rf(p.U)), sub(rf(q.V), rf(p.V))
		lengthOne := add(abs(dx), abs(dy))
		k := abs(sub(mul(dx, rf(p.V)), mul(dy, rf(p.U))))
		leg := mul(d, add(mul(lengthOne, h), mul(k, m)))
		sum = add(sum, add(leg, mul(qMax, mul(k, dPrime))))
	}
	return sum
}

func buildMesh(ctx context.Context, points []sectionrecord.Point2, origin r3.Vec,
	h, a, b *big.Rat, n int) ([]r3.Vec, [][3]int, []int, []*big.Rat, error) {
	q := make([]*big.Rat, n+1)
	verts := make([]r3.Vec, (n+1)*len(points))
	for i := range q {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, nil, err
		}
		t := div(rint(int64(i)), rint(int64(n)))
		q[i] = quadraticAt(a, b, t)
		z := add(rf(origin.Z), mul(h, t))
		zf, err := exactFloat(z)
		if err != nil {
			return nil, nil, nil, nil, err
		}
		for j, p := range points {
			xf, err := exactFloat(add(rf(origin.X), mul(q[i], rf(p.U))))
			if err != nil {
				return nil, nil, nil, nil, err
			}
			yf, err := exactFloat(add(rf(origin.Y), mul(q[i], rf(p.V))))
			if err != nil {
				return nil, nil, nil, nil, err
			}
			verts[i*len(points)+j] = r3.Vec{X: xf, Y: yf, Z: zf}
		}
	}
	tris := make([][3]int, 0, 2*n*len(points)+2*(len(points)-2))
	sources := make([]int, 0, cap(tris))
	for band := range n {
		for j := range points {
			jn := (j + 1) % len(points)
			a0, b0 := band*len(points)+j, band*len(points)+jn
			a1, b1 := (band+1)*len(points)+j, (band+1)*len(points)+jn
			tris = append(tris, [3]int{a0, b0, b1}, [3]int{a0, b1, a1})
			sources = append(sources, j, j)
		}
	}
	index := make([]int, len(points))
	for i := range index {
		index[i] = i
	}
	capTris, err := triangulation.Triangulate(ctx, points, [][]int{index})
	if err != nil {
		return nil, nil, nil, nil, triangulation.WrapLoftError(err)
	}
	for _, tri := range capTris {
		tris = append(tris, [3]int{tri[0], tri[2], tri[1]})
		sources = append(sources, len(points))
		base := n * len(points)
		tris = append(tris, [3]int{base + tri[0], base + tri[1], base + tri[2]})
		sources = append(sources, len(points)+1)
	}
	return verts, tris, sources, q, nil
}

func BuildThree(ctx context.Context, profiles [3]momentinput.Profile,
	planes [3]sectionrecord.PlaneRecord) (Build, error) {
	if err := ctx.Err(); err != nil {
		return Build{}, err
	}
	var points [3][]sectionrecord.Point2
	for i := range profiles {
		p, err := walkedPoints(profiles[i])
		if err != nil {
			return Build{}, err
		}
		points[i] = p
		if !proofbound.FiniteVec(planes[i].Origin) ||
			planes[i].U != (r3.Vec{X: 1}) || planes[i].V != (r3.Vec{Y: 1}) ||
			planes[i].Origin.X != planes[0].Origin.X || planes[i].Origin.Y != planes[0].Origin.Y {
			return Build{}, fmt.Errorf("%w: three-section loft planes must share world-XY axes and origin", decaderr.ErrUnsupported)
		}
	}
	z0, z1, z2 := rf(planes[0].Origin.Z), rf(planes[1].Origin.Z), rf(planes[2].Origin.Z)
	if z0.Cmp(z1) >= 0 || z1.Cmp(z2) >= 0 || mul(rint(2), z1).Cmp(add(z0, z2)) != 0 {
		return Build{}, fmt.Errorf("%w: three-section loft planes need ascending, equally spaced Z levels", decaderr.ErrUnsupported)
	}
	area := areaOf(points[0])
	if area.Sign() <= 0 {
		return Build{}, fmt.Errorf("%w: three-section loft outer loop has no positive area", decaderr.ErrUnsupported)
	}
	if !originInKernel(points[0]) {
		return Build{}, fmt.Errorf("%w: three-section loft scale origin lies outside the profile kernel", decaderr.ErrUnsupported)
	}
	q1, err := scaleOf(points[0], points[1])
	if err != nil {
		return Build{}, err
	}
	q2, err := scaleOf(points[0], points[2])
	if err != nil {
		return Build{}, err
	}
	a := mul(rint(2), add(sub(q2, mul(rint(2), q1)), rint(1)))
	b := sub(sub(q2, rint(1)), a)
	bernsteinMid := sub(mul(rint(2), q1), div(add(rint(1), q2), rint(2)))
	if bernsteinMid.Sign() < 0 {
		return Build{}, fmt.Errorf("%w: three-section loft scale has no positive Bernstein proof", decaderr.ErrUnsupported)
	}
	qMax := maxRat(maxRat(rint(1), bernsteinMid), q2)
	radiusOne := new(big.Rat)
	for _, p := range points[0] {
		radiusOne = maxRat(radiusOne, add(abs(rf(p.U)), abs(rf(p.V))))
	}
	var n int
	var delta *big.Rat
	for n = 16; n <= 128; n *= 2 {
		delta = div(mul(abs(a), radiusOne), rint(int64(4*n*n)))
		if delta.Cmp(big.NewRat(1, 20)) <= 0 {
			break
		}
	}
	if n > 128 {
		return Build{}, fmt.Errorf("%w: three-section loft exceeds its 0.05 mm mesh bound within 128 bands", decaderr.ErrUnsupported)
	}
	h := sub(z2, z0)
	verts, tris, sources, q, err := buildMesh(ctx, points[0], planes[0].Origin, h, a, b, n)
	if err != nil {
		return Build{}, err
	}
	heldIntegral := new(big.Rat)
	for i := range n {
		heldIntegral = add(heldIntegral, add(add(mul(q[i], q[i]), mul(q[i], q[i+1])), mul(q[i+1], q[i+1])))
	}
	heldIntegral = div(heldIntegral, rint(int64(3*n)))
	volumeGap := mul(mul(area, h), abs(sub(squaredIntegral(a, b), heldIntegral)))
	areaGap := areaSlack(points[0], a, b, qMax, h, n)
	diameter := add(mul(rint(2), mul(qMax, radiusOne)), h)
	result := Build{Vertices: verts, Triangles: tris, Sources: sources, EdgeCount: len(points[0]),
		MeshBound: proofbound.RatFloatUp(delta), VolSymDiff: proofbound.RatFloatUp(volumeGap),
		AreaSlack: proofbound.RatFloatUp(areaGap), Diameter: proofbound.RatFloatUp(diameter)}
	for _, v := range [...]float64{result.MeshBound, result.VolSymDiff, result.AreaSlack, result.Diameter} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return Build{}, fmt.Errorf("%w: three-section loft proof exceeds float64", decaderr.ErrUnsupported)
		}
	}
	return result, nil
}
