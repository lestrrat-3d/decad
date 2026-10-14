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

const maxRectangularSections = 32

type rectangularSpan struct {
	width, depth rectangularCubic
	bands        int
}

// BuildManyRectangles constructs a bounded C1 loft from authenticated rectangle records.
// It requires five to maxRectangularSections source sections.
func BuildManyRectangles(ctx context.Context, profiles []momentinput.Profile,
	planes []sectionrecord.PlaneRecord) (Build, error) {
	if len(profiles) < 5 || len(profiles) > maxRectangularSections || len(planes) != len(profiles) {
		return Build{}, fmt.Errorf("%w: rectangular loft needs 5 to %d matching sections",
			decaderr.ErrUnsupported, maxRectangularSections)
	}
	if err := ctx.Err(); err != nil {
		return Build{}, err
	}
	sections := make([]rectangularSection, len(profiles))
	for i, profile := range profiles {
		section, ok := rectangularRecord(profile)
		if !ok {
			return Build{}, fmt.Errorf("%w: multi-section loft needs centered whole-line rectangles",
				decaderr.ErrUnsupported)
		}
		sections[i] = section
		if !proofbound.FiniteVec(planes[i].Origin) ||
			planes[i].U != (r3.Vec{X: 1}) || planes[i].V != (r3.Vec{Y: 1}) ||
			planes[i].Origin.X != planes[0].Origin.X || planes[i].Origin.Y != planes[0].Origin.Y {
			return Build{}, fmt.Errorf("%w: rectangular loft planes must share world-XY axes and origin",
				decaderr.ErrUnsupported)
		}
	}
	z0 := rf(planes[0].Origin.Z)
	step := sub(rf(planes[1].Origin.Z), z0)
	if step.Sign() <= 0 {
		return Build{}, fmt.Errorf("%w: rectangular loft planes need ascending, equally spaced Z levels",
			decaderr.ErrUnsupported)
	}
	for i := 2; i < len(planes); i++ {
		if rf(planes[i].Origin.Z).Cmp(add(z0, mul(rint(int64(i)), step))) != 0 {
			return Build{}, fmt.Errorf("%w: rectangular loft planes need ascending, equally spaced Z levels",
				decaderr.ErrUnsupported)
		}
	}
	widths, depths := make([]*big.Rat, len(sections)), make([]*big.Rat, len(sections))
	for i, section := range sections {
		widths[i], depths[i] = section.width, section.height
	}
	widthSlopes := rectangularTangents(widths)
	depthSlopes := rectangularTangents(depths)
	spans := make([]rectangularSpan, len(sections)-1)
	meshBound, volumeGap, areaGap := rint(0), rint(0), rint(0)
	maximumWidth, maximumDepth := rint(0), rint(0)
	totalBands := 0
	for i := range spans {
		if err := ctx.Err(); err != nil {
			return Build{}, err
		}
		w, err := rectangularHermiteSpan(widths[i], widths[i+1], widthSlopes[i], widthSlopes[i+1])
		if err != nil {
			return Build{}, err
		}
		h, err := rectangularHermiteSpan(depths[i], depths[i+1], depthSlopes[i], depthSlopes[i+1])
		if err != nil {
			return Build{}, err
		}
		spans[i].width, spans[i].depth = w, h
		curvature := add(w.curvatureUpper, h.curvatureUpper)
		var delta *big.Rat
		for bands := 16; bands <= 128; bands *= 2 {
			delta = div(curvature, rint(int64(8*bands*bands)))
			if delta.Cmp(big.NewRat(1, 20)) <= 0 {
				spans[i].bands = bands
				break
			}
		}
		if spans[i].bands == 0 {
			return Build{}, fmt.Errorf("%w: rectangular loft span %d exceeds the 0.05 mm mesh bound",
				decaderr.ErrUnsupported, i)
		}
		totalBands += spans[i].bands
		meshBound = maxRat(meshBound, delta)
		maximumWidth, maximumDepth = maxRat(maximumWidth, w.maximum), maxRat(maximumDepth, h.maximum)
		nn := rint(int64(spans[i].bands))
		meanW := div(w.curvatureUpper, mul(rint(12), mul(nn, nn)))
		meanH := div(h.curvatureUpper, mul(rint(12), mul(nn, nn)))
		slopeW := div(w.curvatureUpper, mul(rint(2), nn))
		slopeH := div(h.curvatureUpper, mul(rint(2), nn))
		volumeGap = add(volumeGap, mul(mul(rint(4), step),
			add(mul(h.maximum, meanW), mul(w.maximum, meanH))))
		areaGap = add(areaGap, mul(rint(4), add(add(
			mul(add(step, h.slopeMaximum), meanW), mul(w.maximum, slopeH)), add(
			mul(add(step, w.slopeMaximum), meanH), mul(h.maximum, slopeW)))))
	}
	vertices, triangles, sources, err := rectangularSpanMesh(ctx, sections[0].points, planes[0].Origin,
		step, spans, totalBands)
	if err != nil {
		return Build{}, err
	}
	height := mul(rint(int64(len(spans))), step)
	diameter := add(mul(rint(2), add(maximumWidth, maximumDepth)), height)
	result := Build{Vertices: vertices, Triangles: triangles, Sources: sources, EdgeCount: 4,
		MeshBound: proofbound.RatFloatUp(meshBound), VolSymDiff: proofbound.RatFloatUp(volumeGap),
		AreaSlack: proofbound.RatFloatUp(areaGap), Diameter: proofbound.RatFloatUp(diameter),
		RectangularSection: manyRectangularSection(planes[0].Origin, step, spans)}
	for _, value := range [...]float64{result.MeshBound, result.VolSymDiff, result.AreaSlack, result.Diameter} {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return Build{}, fmt.Errorf("%w: rectangular section proof exceeds float64", decaderr.ErrUnsupported)
		}
	}
	return result, nil
}

func rectangularTangents(values []*big.Rat) []*big.Rat {
	tangents := make([]*big.Rat, len(values))
	tangents[0] = sub(values[1], values[0])
	for i := 1; i < len(values)-1; i++ {
		tangents[i] = div(sub(values[i+1], values[i-1]), rint(2))
	}
	tangents[len(values)-1] = sub(values[len(values)-1], values[len(values)-2])
	return tangents
}

func rectangularHermiteSpan(start, end, startSlope, endSlope *big.Rat) (rectangularCubic, error) {
	controls := [4]*big.Rat{start, add(start, div(startSlope, rint(3))),
		sub(end, div(endSlope, rint(3))), end}
	if controls[0].Sign() <= 0 || controls[3].Sign() <= 0 ||
		controls[1].Sign() < 0 || controls[2].Sign() < 0 {
		return rectangularCubic{}, fmt.Errorf("%w: rectangular loft has no positive cubic proof",
			decaderr.ErrUnsupported)
	}
	return rectangularCubic{controls: controls, maximum: fourCubicMaximum(controls),
		slopeMaximum: fourCubicSlopeMaximum(controls), curvatureUpper: fourCubicCurvature(controls)}, nil
}

func rectangularSpanMesh(ctx context.Context, points []sectionrecord.Point2, origin r3.Vec,
	step *big.Rat, spans []rectangularSpan, totalBands int) ([]r3.Vec, [][3]int, []int, error) {
	vertices := make([]r3.Vec, (totalBands+1)*len(points))
	ring := 0
	for i, span := range spans {
		for j := 0; j <= span.bands; j++ {
			if i != 0 && j == 0 {
				continue
			}
			if err := ctx.Err(); err != nil {
				return nil, nil, nil, err
			}
			t := div(rint(int64(j)), rint(int64(span.bands)))
			w, h := span.width.at(t), span.depth.at(t)
			z, err := exactFloat(add(rf(origin.Z), mul(step, add(rint(int64(i)), t))))
			if err != nil {
				return nil, nil, nil, err
			}
			for k, point := range points {
				x, y := w, h
				if point.U < 0 {
					x = new(big.Rat).Neg(x)
				}
				if point.V < 0 {
					y = new(big.Rat).Neg(y)
				}
				xf, err := exactFloat(add(rf(origin.X), x))
				if err != nil {
					return nil, nil, nil, err
				}
				yf, err := exactFloat(add(rf(origin.Y), y))
				if err != nil {
					return nil, nil, nil, err
				}
				vertices[ring*len(points)+k] = r3.Vec{X: xf, Y: yf, Z: z}
			}
			ring++
		}
	}
	triangles := make([][3]int, 0, 2*totalBands*len(points)+2*(len(points)-2))
	sources := make([]int, 0, cap(triangles))
	for band := range totalBands {
		for j := range points {
			next := (j + 1) % len(points)
			a0, b0 := band*len(points)+j, band*len(points)+next
			a1, b1 := (band+1)*len(points)+j, (band+1)*len(points)+next
			triangles = append(triangles, [3]int{a0, b0, b1}, [3]int{a0, b1, a1})
			sources = append(sources, j, j)
		}
	}
	capTris, err := triangulation.Triangulate(ctx, points, [][]int{{0, 1, 2, 3}})
	if err != nil {
		return nil, nil, nil, triangulation.WrapLoftError(err)
	}
	for _, tri := range capTris {
		triangles = append(triangles, [3]int{tri[0], tri[2], tri[1]})
		sources = append(sources, len(points))
		end := totalBands * len(points)
		triangles = append(triangles, [3]int{end + tri[0], end + tri[1], end + tri[2]})
		sources = append(sources, len(points)+1)
	}
	return vertices, triangles, sources, nil
}
