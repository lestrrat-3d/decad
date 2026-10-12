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

type roundedSection struct {
	width  *big.Rat
	height *big.Rat
	radius *big.Rat
	start  int
}

// roundedRecord identifies the eight exact whole records of one rounded
// rectangle. Sketch has already authenticated the profile and its area.
func roundedRecord(profile momentinput.Profile) (roundedSection, error) {
	var result roundedSection
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 8 {
		return result, fmt.Errorf("%w: four-section rounded loft needs one eight-edge outer loop", decaderr.ErrUnsupported)
	}
	segments := profile.Outer.Segments
	for start := range segments {
		bottom, ok := segments[start].(sectionrecord.LineSeg)
		if !ok || bottom.TStart != 0 || bottom.TEnd != 1 ||
			!roundedFinite(bottom.Start.U, bottom.Start.V, bottom.End.U, bottom.End.V) ||
			bottom.Start.V >= 0 || bottom.Start.V != bottom.End.V || bottom.Start.U >= bottom.End.U {
			continue
		}
		corner, ok := segments[(start+1)%8].(sectionrecord.ArcSeg)
		if !ok || corner.TStart != 0 || corner.TEnd != 1 || corner.Start != bottom.End ||
			!roundedFinite(corner.End.U) {
			continue
		}
		w, h := rf(corner.End.U), abs(rf(bottom.Start.V))
		r := sub(w, rf(corner.Start.U))
		if r.Cmp(rint(1)) < 0 || r.Cmp(h) >= 0 || r.Cmp(w) >= 0 ||
			h.Cmp(rint(32)) > 0 || w.Cmp(rint(32)) > 0 {
			continue
		}
		left, right := sub(r, w), sub(w, r)
		low, high := new(big.Rat).Neg(h), h
		p := [8]roundedPoint{
			{left, low}, {right, low}, {w, add(low, r)}, {w, sub(h, r)},
			{right, high}, {left, high}, {new(big.Rat).Neg(w), sub(h, r)},
			{new(big.Rat).Neg(w), add(low, r)},
		}
		centers := [4]roundedPoint{
			{right, add(low, r)}, {right, sub(h, r)},
			{left, sub(h, r)}, {left, add(low, r)},
		}
		valid := true
		for j := range 8 {
			raw := segments[(start+j)%8]
			if j%2 == 0 {
				line, lineOK := raw.(sectionrecord.LineSeg)
				valid = lineOK && line.TStart == 0 && line.TEnd == 1 &&
					roundedPointEqual(line.Start, p[j]) && roundedPointEqual(line.End, p[(j+1)%8])
			} else {
				arc, arcOK := raw.(sectionrecord.ArcSeg)
				valid = arcOK && arc.TStart == 0 && arc.TEnd == 1 &&
					roundedPointEqual(arc.Center, centers[j/2]) &&
					roundedPointEqual(arc.Start, p[j]) && roundedPointEqual(arc.End, p[(j+1)%8])
			}
			if !valid {
				break
			}
		}
		if valid {
			return roundedSection{width: w, height: h, radius: r, start: start}, nil
		}
	}
	return result, fmt.Errorf("%w: four-section rounded loft needs exact counterclockwise lines and quarter arcs",
		decaderr.ErrUnsupported)
}

type roundedPoint struct{ x, y *big.Rat }

func roundedFinite(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func roundedPointEqual(actual sectionrecord.Point2, expected roundedPoint) bool {
	return roundedFinite(actual.U, actual.V) &&
		rf(actual.U).Cmp(expected.x) == 0 && rf(actual.V).Cmp(expected.y) == 0
}

func roundedWidthControls(w [4]*big.Rat) ([4]*big.Rat, error) {
	var b [4]*big.Rat
	b[0], b[3] = w[0], w[3]
	// At t=1/3 and 2/3, the Bernstein weights are respectively
	// (8,12,6,1)/27 and (1,6,12,8)/27.
	a := div(sub(sub(mul(rint(27), w[1]), mul(rint(8), w[0])), w[3]), rint(6))
	c := div(sub(sub(mul(rint(27), w[2]), w[0]), mul(rint(8), w[3])), rint(6))
	b[1] = div(sub(mul(rint(2), a), c), rint(3))
	b[2] = div(sub(mul(rint(2), c), a), rint(3))
	if mul(rint(3), w[1]).Cmp(add(mul(rint(2), w[0]), w[3])) == 0 &&
		mul(rint(3), w[2]).Cmp(add(w[0], mul(rint(2), w[3]))) == 0 {
		return b, fmt.Errorf("%w: four rounded sections need non-affine widths", decaderr.ErrUnsupported)
	}
	increasing := true
	decreasing := true
	for i := range 3 {
		increasing = increasing && b[i].Cmp(b[i+1]) <= 0
		decreasing = decreasing && b[i].Cmp(b[i+1]) >= 0
	}
	if !increasing && !decreasing {
		return b, fmt.Errorf("%w: four rounded sections need monotone width controls", decaderr.ErrUnsupported)
	}
	return b, nil
}

func roundedCubicAt(b [4]*big.Rat, t *big.Rat) *big.Rat {
	u := sub(rint(1), t)
	return add(add(mul(b[0], mul(mul(u, u), u)),
		mul(rint(3), mul(b[1], mul(mul(u, u), t)))),
		add(mul(rint(3), mul(b[2], mul(u, mul(t, t)))), mul(b[3], mul(mul(t, t), t))))
}

func roundedWidthCurvature(b [4]*big.Rat) *big.Rat {
	first := add(sub(b[0], mul(rint(2), b[1])), b[2])
	second := add(sub(b[1], mul(rint(2), b[2])), b[3])
	return mul(rint(6), maxRat(abs(first), abs(second)))
}

// BuildFourRounded certifies a cubic-width rounded rectangle from four
// authenticated Sketch records. It gives the faceted body both a two-sided
// boundary bound and a non-cancelling occupied-volume difference.
func BuildFourRounded(ctx context.Context, profiles [4]momentinput.Profile,
	planes [4]sectionrecord.PlaneRecord) (Build, error) {
	if err := ctx.Err(); err != nil {
		return Build{}, err
	}
	var sections [4]roundedSection
	var widths [4]*big.Rat
	for i := range sections {
		section, err := roundedRecord(profiles[i])
		if err != nil {
			return Build{}, err
		}
		sections[i], widths[i] = section, section.width
		if section.height.Cmp(sections[0].height) != 0 || section.radius.Cmp(sections[0].radius) != 0 {
			return Build{}, fmt.Errorf("%w: four rounded sections need a common height and radius", decaderr.ErrUnsupported)
		}
		if !proofbound.FiniteVec(planes[i].Origin) || planes[i].U != (r3.Vec{X: 1}) ||
			planes[i].V != (r3.Vec{Y: 1}) || planes[i].Origin.X != planes[0].Origin.X ||
			planes[i].Origin.Y != planes[0].Origin.Y {
			return Build{}, fmt.Errorf("%w: four rounded section planes need common world-XY axes and origin",
				decaderr.ErrUnsupported)
		}
	}
	z0 := rf(planes[0].Origin.Z)
	step := sub(rf(planes[1].Origin.Z), z0)
	if step.Sign() <= 0 || mul(rint(3), step).Cmp(rint(32)) > 0 {
		return Build{}, fmt.Errorf("%w: four rounded section levels need a positive height up to 32 mm",
			decaderr.ErrUnsupported)
	}
	for i := 2; i < 4; i++ {
		want := add(z0, mul(rint(int64(i)), step))
		if rf(planes[i].Origin.Z).Cmp(want) != 0 {
			return Build{}, fmt.Errorf("%w: four rounded section levels must be equally spaced",
				decaderr.ErrUnsupported)
		}
	}
	b, err := roundedWidthControls(widths)
	if err != nil {
		return Build{}, err
	}
	r, h := sections[0].radius, sections[0].height
	for _, control := range b {
		if control.Cmp(r) <= 0 || control.Cmp(rint(32)) > 0 {
			return Build{}, fmt.Errorf("%w: four rounded width controls leave the size range", decaderr.ErrUnsupported)
		}
	}
	unit, err := roundedUnitPolygon(ctx)
	if err != nil {
		return Build{}, err
	}
	arcGap := div(r, rint(1024))
	curvature := roundedWidthCurvature(b)
	var bands int
	var widthGap, meshGap *big.Rat
	for bands = 16; bands <= 128; bands *= 2 {
		widthGap = div(curvature, rint(int64(8*bands*bands)))
		meshGap = add(widthGap, arcGap)
		if meshGap.Cmp(big.NewRat(1, 20)) <= 0 {
			break
		}
	}
	if bands > 128 {
		return Build{}, fmt.Errorf("%w: four rounded sections exceed the 0.05 mm mesh bound",
			decaderr.ErrUnsupported)
	}
	vertices, triangles, sources, err := roundedMesh(ctx, unit, b, h, r, planes[0].Origin,
		mul(rint(3), step), bands, sections[0].start)
	if err != nil {
		return Build{}, err
	}
	polygonArea := areaOf(unit)
	cornerGap := mul(mul(r, r), sub(proofbound.PiUpper, polygonArea))
	if cornerGap.Sign() < 0 {
		return Build{}, fmt.Errorf("%w: rounded corner polygon exceeds its source disk", decaderr.ErrUnsupported)
	}
	volGap := mul(mul(rint(3), step), add(mul(mul(rint(4), h), widthGap), cornerGap))
	wMax := maxRat(b[0], b[3])
	maxSlope := new(big.Rat)
	for i := range 3 {
		maxSlope = maxRat(maxSlope, mul(rint(3), abs(sub(b[i+1], b[i]))))
	}
	perimeter := add(mul(rint(4), sub(add(wMax, h), mul(rint(2), r))),
		mul(rint(2), mul(proofbound.PiUpper, r)))
	wallArea := mul(perimeter, add(mul(rint(3), step), maxSlope))
	capArea := mul(rint(2), add(mul(rint(4), mul(wMax, h)),
		mul(proofbound.PiUpper, mul(r, r))))
	heldArea := 0.0
	for _, tri := range triangles {
		heldArea = proofbound.AbsSumUpper(heldArea,
			proofbound.PerturbedTriangleAreaUpper(vertices[tri[0]], vertices[tri[1]], vertices[tri[2]], 0))
	}
	result := Build{Vertices: vertices, Triangles: triangles, Sources: sources, EdgeCount: 8,
		MeshBound: proofbound.RatFloatUp(meshGap), VolSymDiff: proofbound.RatFloatUp(volGap),
		AreaSlack: proofbound.AbsSumUpper(proofbound.RatFloatUp(add(wallArea, capArea)), heldArea),
		Diameter:  proofbound.RatFloatUp(add(mul(rint(2), add(wMax, h)), mul(rint(3), step)))}
	for _, v := range [...]float64{result.MeshBound, result.VolSymDiff, result.AreaSlack, result.Diameter} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return Build{}, fmt.Errorf("%w: four rounded section proof exceeds float64", decaderr.ErrUnsupported)
		}
	}
	return result, nil
}
