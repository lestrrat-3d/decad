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

type rectangularSection struct {
	points        []sectionrecord.Point2
	width, height *big.Rat
}

// rectangularRecord checks the whole recorded walk, not fitted corners. A
// different first corner is harmless because the section denotes the same
// centered rectangle at its own level.
func rectangularRecord(profile momentinput.Profile) (rectangularSection, bool) {
	points, err := walkedPoints(profile)
	if err != nil || len(points) != 4 {
		return rectangularSection{}, false
	}
	w, h := abs(rf(points[0].U)), abs(rf(points[0].V))
	if w.Sign() <= 0 || h.Sign() <= 0 || areaOf(points).Sign() <= 0 {
		return rectangularSection{}, false
	}
	var seen [4]bool
	for i, point := range points {
		if abs(rf(point.U)).Cmp(w) != 0 || abs(rf(point.V)).Cmp(h) != 0 {
			return rectangularSection{}, false
		}
		corner := 0
		if point.U > 0 {
			corner += 2
		}
		if point.V > 0 {
			corner++
		}
		if seen[corner] {
			return rectangularSection{}, false
		}
		seen[corner] = true
		next := points[(i+1)%len(points)]
		if (point.U == next.U) == (point.V == next.V) {
			return rectangularSection{}, false
		}
	}
	return rectangularSection{points: points, width: w, height: h}, true
}

func rectangularHomothetic(sections [3]rectangularSection) bool {
	for i := 1; i < len(sections); i++ {
		if mul(sections[i].width, sections[0].height).Cmp(
			mul(sections[i].height, sections[0].width)) != 0 {
			return false
		}
	}
	return true
}

func rectangularWalksMatch(sections [3]rectangularSection) bool {
	for i := 1; i < len(sections); i++ {
		for j, base := range sections[0].points {
			other := sections[i].points[j]
			if (base.U > 0) != (other.U > 0) || (base.V > 0) != (other.V > 0) {
				return false
			}
		}
	}
	return true
}

type rectangularQuadratic struct {
	start, middle, end *big.Rat
	a, b, maximum      *big.Rat
	slopeMaximum       *big.Rat
}

func rectangularWidth(values [3]*big.Rat) (rectangularQuadratic, error) {
	start, middle, end := values[0], values[1], values[2]
	if start.Sign() <= 0 || middle.Sign() <= 0 || end.Sign() <= 0 {
		return rectangularQuadratic{}, fmt.Errorf("%w: rectangular loft width or height is not positive",
			decaderr.ErrUnsupported)
	}
	control := sub(mul(rint(2), middle), div(add(start, end), rint(2)))
	if control.Sign() < 0 {
		return rectangularQuadratic{}, fmt.Errorf("%w: rectangular loft width or height has no positive Bernstein proof",
			decaderr.ErrUnsupported)
	}
	a := mul(rint(2), add(sub(end, mul(rint(2), middle)), start))
	b := sub(sub(end, start), a)
	return rectangularQuadratic{start: start, middle: middle, end: end, a: a, b: b,
		maximum:      maxRat(maxRat(start, control), end),
		slopeMaximum: maxRat(abs(b), abs(add(mul(rint(2), a), b)))}, nil
}

func (q rectangularQuadratic) at(t *big.Rat) *big.Rat {
	return add(q.start, add(mul(q.b, t), mul(q.a, mul(t, t))))
}

// TryBuildThreeRectangles claims centered rectangles whose dimensions vary
// independently or whose walks start at different corners. Every later
// admission and proof gate belongs to this builder. Aligned homothetic walks
// and other profiles continue through BuildThree unchanged.
func TryBuildThreeRectangles(ctx context.Context, profiles [3]momentinput.Profile,
	planes [3]sectionrecord.PlaneRecord) (Build, bool, error) {
	var sections [3]rectangularSection
	for i, profile := range profiles {
		section, ok := rectangularRecord(profile)
		if !ok {
			return Build{}, false, nil
		}
		sections[i] = section
	}
	if rectangularHomothetic(sections) && rectangularWalksMatch(sections) {
		return Build{}, false, nil
	}
	built, err := buildThreeRectangles(ctx, sections, planes)
	return built, true, err
}

func buildThreeRectangles(ctx context.Context, sections [3]rectangularSection,
	planes [3]sectionrecord.PlaneRecord) (Build, error) {
	if err := ctx.Err(); err != nil {
		return Build{}, err
	}
	for i := range planes {
		if !proofbound.FiniteVec(planes[i].Origin) ||
			planes[i].U != (r3.Vec{X: 1}) || planes[i].V != (r3.Vec{Y: 1}) ||
			planes[i].Origin.X != planes[0].Origin.X || planes[i].Origin.Y != planes[0].Origin.Y {
			return Build{}, fmt.Errorf("%w: rectangular loft planes must share world-XY axes and origin",
				decaderr.ErrUnsupported)
		}
	}
	z0, z1, z2 := rf(planes[0].Origin.Z), rf(planes[1].Origin.Z), rf(planes[2].Origin.Z)
	if z0.Cmp(z1) >= 0 || z1.Cmp(z2) >= 0 || mul(rint(2), z1).Cmp(add(z0, z2)) != 0 {
		return Build{}, fmt.Errorf("%w: rectangular loft planes need ascending, equally spaced Z levels",
			decaderr.ErrUnsupported)
	}
	height := sub(z2, z0)
	var widths, heights [3]*big.Rat
	for i, section := range sections {
		widths[i], heights[i] = section.width, section.height
	}
	w, err := rectangularWidth(widths)
	if err != nil {
		return Build{}, err
	}
	h, err := rectangularWidth(heights)
	if err != nil {
		return Build{}, err
	}
	var bands int
	var delta *big.Rat
	for bands = 16; bands <= 128; bands *= 2 {
		delta = div(add(abs(w.a), abs(h.a)), rint(int64(4*bands*bands)))
		if delta.Cmp(big.NewRat(1, 20)) <= 0 {
			break
		}
	}
	if bands > 128 {
		return Build{}, fmt.Errorf("%w: rectangular loft exceeds its 0.05 mm bound within 128 bands",
			decaderr.ErrUnsupported)
	}
	vertices, triangles, sources, err := rectangularMesh(ctx, sections[0].points, planes[0].Origin,
		height, w, h, bands)
	if err != nil {
		return Build{}, err
	}
	nn := rint(int64(bands))
	meanWidthGap := div(abs(w.a), mul(rint(6), mul(nn, nn)))
	meanHeightGap := div(abs(h.a), mul(rint(6), mul(nn, nn)))
	meanWidthSlopeGap := div(abs(w.a), mul(rint(2), nn))
	meanHeightSlopeGap := div(abs(h.a), mul(rint(2), nn))
	volumeGap := mul(mul(rint(4), height), add(
		mul(h.maximum, meanWidthGap), mul(w.maximum, meanHeightGap)))
	areaGap := mul(rint(4), add(add(
		mul(add(height, h.slopeMaximum), meanWidthGap),
		mul(w.maximum, meanHeightSlopeGap)), add(
		mul(add(height, w.slopeMaximum), meanHeightGap),
		mul(h.maximum, meanWidthSlopeGap))))
	diameter := add(mul(rint(2), add(w.maximum, h.maximum)), height)
	result := Build{Vertices: vertices, Triangles: triangles, Sources: sources, EdgeCount: 4,
		MeshBound: proofbound.RatFloatUp(delta), VolSymDiff: proofbound.RatFloatUp(volumeGap),
		AreaSlack: proofbound.RatFloatUp(areaGap), Diameter: proofbound.RatFloatUp(diameter),
		RectangularSection: threeRectangularSection(planes[0].Origin, planes[2].Origin, w, h)}
	for _, value := range [...]float64{result.MeshBound, result.VolSymDiff, result.AreaSlack, result.Diameter} {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return Build{}, fmt.Errorf("%w: rectangular loft proof exceeds float64", decaderr.ErrUnsupported)
		}
	}
	return result, nil
}

type rectangularStationDimension interface {
	at(*big.Rat) *big.Rat
}

func rectangularMesh(ctx context.Context, points []sectionrecord.Point2, origin r3.Vec, height *big.Rat,
	w, h rectangularStationDimension, bands int) ([]r3.Vec, [][3]int, []int, error) {
	vertices := make([]r3.Vec, (bands+1)*len(points))
	for i := range bands + 1 {
		if err := ctx.Err(); err != nil {
			return nil, nil, nil, err
		}
		t := div(rint(int64(i)), rint(int64(bands)))
		width, halfHeight := w.at(t), h.at(t)
		z, err := exactFloat(add(rf(origin.Z), mul(height, t)))
		if err != nil {
			return nil, nil, nil, err
		}
		for j, point := range points {
			x, y := width, halfHeight
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
			vertices[i*len(points)+j] = r3.Vec{X: xf, Y: yf, Z: z}
		}
	}
	triangles := make([][3]int, 0, 2*bands*len(points)+2*(len(points)-2))
	sources := make([]int, 0, cap(triangles))
	for band := range bands {
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
		end := bands * len(points)
		triangles = append(triangles, [3]int{end + tri[0], end + tri[1], end + tri[2]})
		sources = append(sources, len(points)+1)
	}
	return vertices, triangles, sources, nil
}
