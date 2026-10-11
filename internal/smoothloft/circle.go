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
	"github.com/lestrrat-3d/r3"
)

const circleSides = 96

// quarterCircleRadius admits only the four whole, counterclockwise cardinal
// arcs of one origin-centered disk. Sketch has already certified its region;
// this exact record check only identifies the shape used by the loft proof.
func quarterCircleRadius(profile momentinput.Profile) (*big.Rat, error) {
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 4 {
		return nil, fmt.Errorf("%w: a varying circular loft needs four whole quarter arcs", decaderr.ErrUnsupported)
	}
	first, ok := profile.Outer.Segments[0].(sectionrecord.ArcSeg)
	if !ok {
		return nil, fmt.Errorf("%w: a varying circular loft needs quarter arcs", decaderr.ErrUnsupported)
	}
	if math.IsNaN(first.Start.U) || math.IsInf(first.Start.U, 0) {
		return nil, fmt.Errorf("%w: varying circular loft radius is not finite", decaderr.ErrUnsupported)
	}
	r := rf(first.Start.U)
	if r.Cmp(rint(1)) < 0 || r.Cmp(rint(32)) > 0 {
		return nil, fmt.Errorf("%w: a varying circular loft radius must be 1 to 32 mm", decaderr.ErrUnsupported)
	}
	rf64, _ := r.Float64()
	cardinal := [4]sectionrecord.Point2{
		{U: rf64}, {V: rf64}, {U: -rf64}, {V: -rf64},
	}
	for i, raw := range profile.Outer.Segments {
		arc, ok := raw.(sectionrecord.ArcSeg)
		if !ok || arc.Center != (sectionrecord.Point2{}) || arc.TStart != 0 || arc.TEnd != 1 ||
			arc.Start != cardinal[i] || arc.End != cardinal[(i+1)%4] {
			return nil, fmt.Errorf("%w: varying circular loft arcs must be whole cardinal quarters", decaderr.ErrUnsupported)
		}
	}
	return r, nil
}

// circlePolygon constructs a dyadic inscribed 96-gon. Trigonometry only
// proposes vertices. Exact rational gates below prove that its boundary lies
// in the unit disk, contains the concentric disk of radius 1-1/1024, and is
// strictly convex. The resulting radial error needs no sampled-curve claim.
func circlePolygon(ctx context.Context, radius *big.Rat) ([]sectionrecord.Point2, error) {
	const grid = int64(1 << 20)
	unit := make([]sectionrecord.Point2, circleSides)
	for i := range unit {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sine, cosine := math.Sincos(2 * math.Pi * float64(i) / circleSides)
		x := int64(math.Round((1 - 1.0/4096) * cosine * float64(grid)))
		y := int64(math.Round((1 - 1.0/4096) * sine * float64(grid)))
		unit[i] = sectionrecord.Point2{U: float64(x) / float64(grid), V: float64(y) / float64(grid)}
	}
	inner := big.NewRat(1023, 1024)
	for i, p := range unit {
		q, next := unit[(i+1)%len(unit)], unit[(i+2)%len(unit)]
		px, py := rf(p.U), rf(p.V)
		// Source group j/24 is one cardinal quarter arc. Exact quadrant
		// and seam tests keep every held edge with its named source arc.
		switch i / (circleSides / 4) {
		case 0:
			if px.Sign() < 0 || py.Sign() < 0 || (i == 0 && py.Sign() != 0) {
				return nil, fmt.Errorf("%w: circular loft polygon leaves its source quarter", decaderr.ErrUnsupported)
			}
		case 1:
			if px.Sign() > 0 || py.Sign() < 0 || (i == circleSides/4 && px.Sign() != 0) {
				return nil, fmt.Errorf("%w: circular loft polygon leaves its source quarter", decaderr.ErrUnsupported)
			}
		case 2:
			if px.Sign() > 0 || py.Sign() > 0 || (i == circleSides/2 && py.Sign() != 0) {
				return nil, fmt.Errorf("%w: circular loft polygon leaves its source quarter", decaderr.ErrUnsupported)
			}
		case 3:
			if px.Sign() < 0 || py.Sign() > 0 || (i == 3*circleSides/4 && px.Sign() != 0) {
				return nil, fmt.Errorf("%w: circular loft polygon leaves its source quarter", decaderr.ErrUnsupported)
			}
		}
		qx, qy := rf(q.U), rf(q.V)
		dx, dy := sub(qx, px), sub(qy, py)
		cross := sub(mul(px, qy), mul(py, qx))
		turn := sub(mul(dx, sub(rf(next.V), qy)), mul(dy, sub(rf(next.U), qx)))
		if add(mul(px, px), mul(py, py)).Cmp(rint(1)) > 0 || cross.Sign() <= 0 ||
			turn.Sign() <= 0 || mul(cross, cross).Cmp(mul(mul(inner, inner), add(mul(dx, dx), mul(dy, dy)))) < 0 {
			return nil, fmt.Errorf("%w: varying circular loft polygon has no exact radial enclosure", decaderr.ErrUnsupported)
		}
	}
	points := make([]sectionrecord.Point2, len(unit))
	for i, p := range unit {
		x, err := exactFloat(mul(radius, rf(p.U)))
		if err != nil {
			return nil, err
		}
		y, err := exactFloat(mul(radius, rf(p.V)))
		if err != nil {
			return nil, err
		}
		points[i] = sectionrecord.Point2{U: x, V: y}
	}
	return points, nil
}

// BuildThreeCircles certifies a linearly varying circular loft. The three
// actual Sketch records are the disks at t=0, 1/2 and 1. A radius midpoint
// identity makes the wall a ruled cone; the held polygon wall is likewise
// ruled, so all remaining error is the radial polygon-to-circle gap.
func BuildThreeCircles(ctx context.Context, profiles [3]momentinput.Profile,
	planes [3]sectionrecord.PlaneRecord) (Build, error) {
	if err := ctx.Err(); err != nil {
		return Build{}, err
	}
	var radii [3]*big.Rat
	for i := range profiles {
		var err error
		radii[i], err = quarterCircleRadius(profiles[i])
		if err != nil {
			return Build{}, err
		}
		if !proofbound.FiniteVec(planes[i].Origin) ||
			planes[i].U != (r3.Vec{X: 1}) || planes[i].V != (r3.Vec{Y: 1}) ||
			planes[i].Origin.X != planes[0].Origin.X || planes[i].Origin.Y != planes[0].Origin.Y {
			return Build{}, fmt.Errorf("%w: varying circular loft planes must share world-XY axes and origin", decaderr.ErrUnsupported)
		}
	}
	z0, z1, z2 := rf(planes[0].Origin.Z), rf(planes[1].Origin.Z), rf(planes[2].Origin.Z)
	if z0.Cmp(z1) >= 0 || z1.Cmp(z2) >= 0 || mul(rint(2), z1).Cmp(add(z0, z2)) != 0 {
		return Build{}, fmt.Errorf("%w: varying circular loft planes need ascending, equally spaced levels", decaderr.ErrUnsupported)
	}
	h := sub(z2, z0)
	if h.Cmp(rint(32)) > 0 || radii[0].Cmp(radii[2]) == 0 ||
		mul(rint(2), radii[1]).Cmp(add(radii[0], radii[2])) != 0 ||
		mul(rint(2), radii[2]).Cmp(radii[0]) < 0 || radii[2].Cmp(mul(rint(2), radii[0])) > 0 {
		return Build{}, fmt.Errorf("%w: varying circular loft needs a positive linear radius within its size bounds", decaderr.ErrUnsupported)
	}
	points, err := circlePolygon(ctx, radii[0])
	if err != nil {
		return Build{}, err
	}
	q2 := div(radii[2], radii[0])
	b := sub(q2, rint(1))
	verts, tris, sources, _, err := buildMesh(ctx, points, planes[0].Origin, h, rint(0), b, 2)
	if err != nil {
		return Build{}, err
	}
	for i, source := range sources {
		if source < circleSides {
			sources[i] = source / (circleSides / 4)
		} else {
			sources[i] = 4 + source - circleSides
		}
	}

	// The polygon contains an inner disk and lies inside the source disk.
	// Every intermediate section remains nested because its radius is linear.
	// Its exact occupied-volume difference is (pi*r²-Apoly)*H*int(q²).
	area := areaOf(points)
	if area.Sign() <= 0 {
		return Build{}, fmt.Errorf("%w: varying circular loft polygon has no area", decaderr.ErrUnsupported)
	}
	outerArea := mul(proofbound.PiUpper, mul(radii[0], radii[0]))
	areaGap := sub(outerArea, area)
	if areaGap.Sign() < 0 {
		return Build{}, fmt.Errorf("%w: varying circular loft polygon exceeds the source disk", decaderr.ErrUnsupported)
	}
	integral := div(add(add(rint(1), q2), mul(q2, q2)), rint(3))
	volumeGap := mul(mul(areaGap, h), integral)
	qMax := maxRat(rint(1), q2)
	delta := div(mul(radii[0], qMax), rint(1024))
	if delta.Cmp(big.NewRat(1, 20)) > 0 {
		return Build{}, fmt.Errorf("%w: varying circular loft exceeds its 0.05 mm mesh bound", decaderr.ErrUnsupported)
	}

	// Convex perimeter monotonicity gives perimeter(P)>=2*pi*(r-r/1024).
	// The polygon's edge support lies in [r-r/1024,r]. Comparing each
	// ruled wall's slant to the circle wall and using sqrt's 1-Lipschitz
	// property gives the lateral allowance below. The cap allowance is
	// the exact polygon deficit at both ends, with pi rounded upward.
	baseDelta := div(radii[0], rint(1024))
	dr := abs(sub(radii[2], radii[0]))
	wallSlack := mul(mul(proofbound.PiUpper, add(rint(1), q2)),
		mul(baseDelta, add(add(h, dr), mul(sub(radii[0], baseDelta), abs(b)))))
	capSlack := mul(areaGap, add(rint(1), mul(q2, q2)))
	areaSlack := add(wallSlack, capSlack)
	diameter := add(mul(rint(2), maxRat(radii[0], radii[2])), h)
	result := Build{Vertices: verts, Triangles: tris, Sources: sources, EdgeCount: 4,
		MeshBound: proofbound.RatFloatUp(delta), VolSymDiff: proofbound.RatFloatUp(volumeGap),
		AreaSlack: proofbound.RatFloatUp(areaSlack), Diameter: proofbound.RatFloatUp(diameter)}
	for _, v := range [...]float64{result.MeshBound, result.VolSymDiff, result.AreaSlack, result.Diameter} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return Build{}, fmt.Errorf("%w: varying circular loft proof exceeds float64", decaderr.ErrUnsupported)
		}
	}
	return result, nil
}
