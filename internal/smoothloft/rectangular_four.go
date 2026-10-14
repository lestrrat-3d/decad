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

type rectangularCubic struct {
	controls       [4]*big.Rat
	maximum        *big.Rat
	slopeMaximum   *big.Rat
	curvatureUpper *big.Rat
}

func fourRectangularDimension(values [4]*big.Rat) (rectangularCubic, error) {
	controls := fourCubicControls(values)
	if controls[0].Sign() <= 0 || controls[3].Sign() <= 0 ||
		controls[1].Sign() < 0 || controls[2].Sign() < 0 {
		return rectangularCubic{}, fmt.Errorf("%w: four rectangular sections have no positive cubic proof",
			decaderr.ErrUnsupported)
	}
	return rectangularCubic{
		controls: controls, maximum: fourCubicMaximum(controls),
		slopeMaximum: fourCubicSlopeMaximum(controls), curvatureUpper: fourCubicCurvature(controls),
	}, nil
}

func (c rectangularCubic) at(t *big.Rat) *big.Rat {
	return fourCubicAt(c.controls, t)
}

// TryBuildFourRectangles claims four authenticated centered rectangle records.
// A different four-section shape keeps the rounded-rectangle route's refusal.
func TryBuildFourRectangles(ctx context.Context, profiles [4]momentinput.Profile,
	planes [4]sectionrecord.PlaneRecord) (Build, bool, error) {
	var sections [4]rectangularSection
	for i, profile := range profiles {
		section, ok := rectangularRecord(profile)
		if !ok {
			return Build{}, false, nil
		}
		sections[i] = section
	}
	built, err := buildFourRectangles(ctx, sections, planes)
	return built, true, err
}

func buildFourRectangles(ctx context.Context, sections [4]rectangularSection,
	planes [4]sectionrecord.PlaneRecord) (Build, error) {
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
	height := mul(rint(3), step)
	var widths, depths [4]*big.Rat
	for i, section := range sections {
		widths[i], depths[i] = section.width, section.height
	}
	w, err := fourRectangularDimension(widths)
	if err != nil {
		return Build{}, err
	}
	h, err := fourRectangularDimension(depths)
	if err != nil {
		return Build{}, err
	}
	curvature := add(w.curvatureUpper, h.curvatureUpper)
	var bands int
	var delta *big.Rat
	for bands = 16; bands <= 128; bands *= 2 {
		delta = div(curvature, rint(int64(8*bands*bands)))
		if delta.Cmp(big.NewRat(1, 20)) <= 0 {
			break
		}
	}
	if bands > 128 {
		return Build{}, fmt.Errorf("%w: four rectangular sections exceed the 0.05 mm mesh bound",
			decaderr.ErrUnsupported)
	}
	vertices, triangles, sources, err := rectangularMesh(ctx, sections[0].points, planes[0].Origin,
		height, w, h, bands)
	if err != nil {
		return Build{}, err
	}
	nn := rint(int64(bands))
	// Linear interpolation on each band has |f-l| <= M/(8N²),
	// integral |f-l| <= M/(12N²), and integral |f'-l'| <= M/(2N).
	meanWidthGap := div(w.curvatureUpper, mul(rint(12), mul(nn, nn)))
	meanDepthGap := div(h.curvatureUpper, mul(rint(12), mul(nn, nn)))
	meanWidthSlopeGap := div(w.curvatureUpper, mul(rint(2), nn))
	meanDepthSlopeGap := div(h.curvatureUpper, mul(rint(2), nn))
	volumeGap := mul(mul(rint(4), height), add(
		mul(h.maximum, meanWidthGap), mul(w.maximum, meanDepthGap)))
	areaGap := mul(rint(4), add(add(
		mul(add(height, h.slopeMaximum), meanWidthGap),
		mul(w.maximum, meanDepthSlopeGap)), add(
		mul(add(height, w.slopeMaximum), meanDepthGap),
		mul(h.maximum, meanWidthSlopeGap))))
	diameter := add(mul(rint(2), add(w.maximum, h.maximum)), height)
	result := Build{Vertices: vertices, Triangles: triangles, Sources: sources, EdgeCount: 4,
		MeshBound: proofbound.RatFloatUp(delta), VolSymDiff: proofbound.RatFloatUp(volumeGap),
		AreaSlack: proofbound.RatFloatUp(areaGap), Diameter: proofbound.RatFloatUp(diameter),
		RectangularSection: fourRectangularSection(planes[0].Origin, planes[3].Origin, w, h)}
	for _, value := range [...]float64{result.MeshBound, result.VolSymDiff, result.AreaSlack, result.Diameter} {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return Build{}, fmt.Errorf("%w: four rectangular section proof exceeds float64",
				decaderr.ErrUnsupported)
		}
	}
	return result, nil
}
