package smoothloft

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// fitBoundaryCell is one shared parameter interval of the source wall.
// Arc cells have no patch: their stationary source is the recorded arc.
type fitBoundaryCell struct {
	patch    fitPatch
	arc      sectionrecord.Point2
	arcNext  sectionrecord.Point2
	arcGap   float64
	arcRound float64
	arcLen   float64
	seg      int
	isArc    bool
}

func fitRefineCells(parts [4]fitSectionPart) ([]fitBoundaryCell, error) {
	const maxCells = 512
	var cells []fitBoundaryCell
	for j, part := range parts {
		if len(part.cells) != 0 {
			for _, initial := range part.cells {
				var split func(fitPatch, int) error
				split = func(p fitPatch, depth int) error {
					if fitPatchUErr(p).Cmp(big.NewRat(1, 200)) <= 0 {
						cells = append(cells, fitBoundaryCell{patch: p, seg: j})
						return nil
					}
					if depth == 12 || len(cells) >= maxCells {
						return fmt.Errorf("%w: fitted loft exceeds its curve-cell bound", decaderr.ErrUnsupported)
					}
					left, right := fitPatchSplitU(p)
					if err := split(left, depth+1); err != nil {
						return err
					}
					return split(right, depth+1)
				}
				if err := split(initial.patch, 0); err != nil {
					return nil, err
				}
			}
			continue
		}
		if len(part.arc) < 1 {
			return nil, fmt.Errorf("%w: fixed arc has no certified stations", decaderr.ErrUnsupported)
		}
		for k := range part.arc {
			next := part.arcSeg.End
			if k+1 < len(part.arc) {
				next = part.arc[k+1]
			}
			cells = append(cells, fitBoundaryCell{
				arc: part.arc[k], arcNext: next, arcGap: part.arcGap,
				arcRound: part.arcRound,
				arcLen:   loftmesh.PerCellArcUpper(part.arcSeg, part.arcWalk, len(part.arc)),
				seg:      j, isArc: true,
			})
		}
	}
	if len(cells) > maxCells {
		return nil, fmt.Errorf("%w: fitted loft exceeds 512 boundary cells", decaderr.ErrUnsupported)
	}
	return cells, nil
}

func fitCellPoint(cell fitBoundaryCell, t *big.Rat, end int) freeform.RatPoint {
	if !cell.isArc {
		return fitPatchAtT(cell.patch, t, end)
	}
	p := cell.arc
	if end == 3 {
		p = cell.arcNext
	}
	return fitPoint(rf(p.U), rf(p.V))
}

func fitCellSpeed(p fitPatch) *big.Rat {
	worst := new(big.Rat)
	for j := range p[0] {
		for i := range 3 {
			worst = maxRat(worst, mul(rint(3), fitPointL1(fitPointSub(p[i+1][j], p[i][j]))))
		}
	}
	return worst
}

func fitCellHeightSpeed(p fitPatch) *big.Rat {
	worst := new(big.Rat)
	for i := range p {
		for j := range 2 {
			worst = maxRat(worst, mul(rint(2), fitPointL1(fitPointSub(p[i][j+1], p[i][j]))))
		}
	}
	return worst
}

func fitCellRadius(p fitPatch) *big.Rat {
	worst := new(big.Rat)
	for i := range p {
		for j := range p[i] {
			worst = maxRat(worst, fitPointL1(p[i][j]))
		}
	}
	return worst
}

func fitCellTwist(p00, p10, p01, p11 freeform.RatPoint) *big.Rat {
	return div(fitPointL1(fitPointSub(fitPointSub(p11, p10), fitPointSub(p01, p00))), rint(4))
}

func fitRingPoint(p freeform.RatPoint, origin r3.Vec, z float64) (r3.Vec, *big.Rat, error) {
	x := add(p.U, rf(origin.X))
	y := add(p.V, rf(origin.Y))
	xf, _ := x.Float64()
	yf, _ := y.Float64()
	if !proofbound.FiniteVec(r3.Vec{X: xf, Y: yf, Z: z}) {
		return r3.Vec{}, nil, fmt.Errorf("%w: fitted loft station exceeds float64", decaderr.ErrUnsupported)
	}
	d := add(abs(sub(x, rf(xf))), abs(sub(y, rf(yf))))
	return r3.Vec{X: xf, Y: yf, Z: z}, d, nil
}

// BuildThreeFits preserves the three Sketch natural-cubic interpolants as a
// quadratic height wall. Its mesh and occupied-volume bound refer to that
// continuum, rather than to a sampled replacement curve.
func BuildThreeFits(ctx context.Context, profiles [3]momentinput.Profile,
	planes [3]sectionrecord.PlaneRecord, work [3]*freeform.FreeformWork) (Build, error) {
	if err := ctx.Err(); err != nil {
		return Build{}, err
	}
	h, err := fitSectionPlanes(planes)
	if err != nil {
		return Build{}, err
	}
	parts, err := fitSectionParts(ctx, profiles, work)
	if err != nil {
		return Build{}, err
	}
	cells, err := fitRefineCells(parts)
	if err != nil {
		return Build{}, err
	}
	var bands int
	var delta *big.Rat
	for bands = 16; bands <= 128; bands *= 2 {
		delta = new(big.Rat)
		for _, cell := range cells {
			if cell.isArc {
				delta = maxRat(delta, rf(proofbound.AbsSumUpper(cell.arcGap, cell.arcRound)))
				continue
			}
			local := add(fitPatchUErr(cell.patch), fitPatchZErr(cell.patch, bands))
			for i := range bands {
				t0 := big.NewRat(int64(i), int64(bands))
				t1 := big.NewRat(int64(i+1), int64(bands))
				p00, p10 := fitCellPoint(cell, t0, 0), fitCellPoint(cell, t0, 3)
				p01, p11 := fitCellPoint(cell, t1, 0), fitCellPoint(cell, t1, 3)
				local = maxRat(local, add(add(fitPatchUErr(cell.patch), fitPatchZErr(cell.patch, bands)),
					fitCellTwist(p00, p10, p01, p11)))
			}
			delta = maxRat(delta, local)
		}
		if delta.Cmp(big.NewRat(49, 1000)) <= 0 {
			break
		}
	}
	if bands > 128 {
		return Build{}, fmt.Errorf("%w: fitted loft exceeds its 0.05 mm mesh bound", decaderr.ErrUnsupported)
	}
	p := len(cells)
	verts := make([]r3.Vec, (bands+1)*p)
	maxRound := new(big.Rat)
	for i := 0; i <= bands; i++ {
		if err := ctx.Err(); err != nil {
			return Build{}, err
		}
		t := big.NewRat(int64(i), int64(bands))
		z, err := exactFloat(add(rf(planes[0].Origin.Z), mul(h, t)))
		if err != nil {
			return Build{}, err
		}
		for j, cell := range cells {
			v, round, err := fitRingPoint(fitCellPoint(cell, t, 0), planes[0].Origin, z)
			if err != nil {
				return Build{}, err
			}
			verts[i*p+j] = v
			maxRound = maxRat(maxRound, round)
		}
	}
	delta = add(delta, maxRound)
	if delta.Cmp(big.NewRat(1, 20)) > 0 {
		return Build{}, fmt.Errorf("%w: fitted loft station rounding exceeds its mesh bound", decaderr.ErrUnsupported)
	}
	tris := make([][3]int, 0, 2*bands*p+2*(p-2))
	sources := make([]int, 0, cap(tris))
	for i := range bands {
		for j, cell := range cells {
			next := (j + 1) % p
			a0, b0 := i*p+j, i*p+next
			a1, b1 := (i+1)*p+j, (i+1)*p+next
			tris = append(tris, [3]int{a0, b0, b1}, [3]int{a0, b1, a1})
			sources = append(sources, cell.seg, cell.seg)
		}
	}
	capPoints := make([]sectionrecord.Point2, p)
	cycle := make([]int, p)
	for j := range cells {
		capPoints[j] = sectionrecord.Point2{U: verts[j].X, V: verts[j].Y}
		cycle[j] = j
	}
	capTris, err := triangulation.Triangulate(ctx, capPoints, [][]int{cycle})
	if err != nil {
		return Build{}, triangulation.WrapLoftError(err)
	}
	for _, tri := range capTris {
		tris = append(tris, [3]int{tri[0], tri[2], tri[1]},
			[3]int{bands*p + tri[0], bands*p + tri[1], bands*p + tri[2]})
		sources = append(sources, 4, 5)
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, tris); err != nil {
		return Build{}, err
	}

	// At each height, match every source boundary point to its held chord.
	// The swept homotopy covers their occupied symmetric difference because
	// both boundaries are simple and keep the same winding. Bounding the
	// homotopy's Jacobian by displacement times tangential speed gives a
	// non-cancelling volume allowance after integration over the height.
	volumeSection := new(big.Rat)
	trueArea := new(big.Rat)
	radius := new(big.Rat)
	for _, cell := range cells {
		if cell.isArc {
			d := rf(proofbound.AbsSumUpper(cell.arcGap, cell.arcRound, proofbound.RatFloatUp(maxRound)))
			chord := fitPointL1(fitPointSub(fitCellPoint(cell, rint(0), 3), fitCellPoint(cell, rint(0), 0)))
			speed := maxRat(add(chord, mul(rint(2), maxRound)), rf(cell.arcLen))
			volumeSection = add(volumeSection, mul(d, speed))
			trueArea = add(trueArea, mul(h, rf(cell.arcLen)))
			radius = maxRat(radius, rf(parts[cell.seg].arcR))
			continue
		}
		speed := fitCellSpeed(cell.patch)
		heightSpeed := fitCellHeightSpeed(cell.patch)
		radius = maxRat(radius, fitCellRadius(cell.patch))
		trueArea = add(trueArea, mul(speed, add(h, heightSpeed)))
		for i := range bands {
			t0, t1 := big.NewRat(int64(i), int64(bands)), big.NewRat(int64(i+1), int64(bands))
			p00, p10 := fitCellPoint(cell, t0, 0), fitCellPoint(cell, t0, 3)
			p01, p11 := fitCellPoint(cell, t1, 0), fitCellPoint(cell, t1, 3)
			chord := maxRat(fitPointL1(fitPointSub(p10, p00)), fitPointL1(fitPointSub(p11, p01)))
			d := add(add(add(fitPatchUErr(cell.patch), fitPatchZErr(cell.patch, bands)),
				fitCellTwist(p00, p10, p01, p11)), maxRound)
			heldSpeed := add(chord, mul(rint(2), maxRound))
			volumeSection = add(volumeSection, div(mul(d, maxRat(speed, heldSpeed)), rint(int64(bands))))
		}
	}
	volumeGap := mul(h, volumeSection)
	// These enclosing areas give a finite, non-cancelling area difference.
	// The occupied-volume certificate above is intentionally much tighter.
	trueArea = add(trueArea, mul(rint(2), mul(proofbound.PiUpper, mul(radius, radius))))
	heldArea := 0.0
	for _, tri := range tris {
		heldArea = proofbound.AbsSumUpper(heldArea,
			proofbound.PerturbedTriangleAreaUpper(verts[tri[0]], verts[tri[1]], verts[tri[2]], 0))
	}
	var radii [4]float64
	for j, part := range parts {
		radii[j] = part.arcR
	}
	result := Build{Vertices: verts, Triangles: tris, Sources: sources, EdgeCount: 4,
		MeshBound: proofbound.RatFloatUp(delta), VolSymDiff: proofbound.RatFloatUp(volumeGap),
		AreaSlack: proofbound.AbsSumUpper(proofbound.RatFloatUp(trueArea), heldArea),
		Diameter:  proofbound.RatFloatUp(add(mul(rint(2), radius), h)), ArcRadii: radii}
	for _, v := range [...]float64{result.MeshBound, result.VolSymDiff, result.AreaSlack, result.Diameter} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return Build{}, fmt.Errorf("%w: fitted loft proof exceeds float64", decaderr.ErrUnsupported)
		}
	}
	return result, nil
}
