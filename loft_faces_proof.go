package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The LoftFaces cap-map carriers live in internal/loftface.
type loftFaceRatVec = loftface.RatVec
type loftFaceAffine = loftface.Affine
type loftFaceMap = loftface.Map
type loftFaceInterval = loftface.Interval

func lfZero() *big.Rat       { return new(big.Rat) }
func lfOne() *big.Rat        { return big.NewRat(1, 1) }
func lfF(v float64) *big.Rat { return ratOfPointPlane(v) }

// lfAdd, lfSub, lfMul and lfDiv return a fresh Rat and read their operands only.
// Operands must be in lowest terms: the result always equals the exact value,
// but is in lowest terms only for reduced operands (a 3/9 operand gives 3/9
// where big.Rat gives 1/3). Every producer in this package reduces: FloatRat
// through lfF and ratOfPointPlane, big.NewRat literals, lfOne, lfZero, and
// results of these helpers and of raw Quo, Abs and Neg. The guard
// TestNoRatWritesThroughNumDenom (internal/proof) bars in-place Num and Denom
// writes outside internal/proof. lfDiv by zero panics as Rat.Quo does.
func lfAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }
func lfSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }
func lfMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }
func lfDiv(a, b *big.Rat) *big.Rat { return proofarith.QuoRat(new(big.Rat), a, b) }
func lfAbs(a *big.Rat) *big.Rat    { return new(big.Rat).Abs(a) }

func lfSimilarity(section loftFaceSection) (loftFaceAffine, error) {
	out := loftface.Identity()
	if section.mesh.pointSimilarity != nil {
		for _, step := range section.mesh.pointSimilarity.steps {
			if step.factor == 0 {
				if !step.motion.IsValid() {
					return loftFaceAffine{}, fmt.Errorf("%w: loft cap has an invalid rigid step", ErrUnsupported)
				}
				out = loftface.Compose(out, loftface.Motion(step.motion))
			} else {
				if !proofbound.FiniteVec(step.anchor) || !proofbound.FiniteVec(r3.Vec{X: step.factor}) ||
					step.factor <= 0 {
					return loftFaceAffine{}, fmt.Errorf("%w: loft cap has an invalid scale step", ErrUnsupported)
				}
				out = loftface.Compose(out, loftface.Scale(step.anchor, step.factor))
			}
		}
	} else if section.mesh.xform != r3.Identity() {
		if !section.mesh.xform.IsValid() {
			return loftFaceAffine{}, fmt.Errorf("%w: loft cap has an invalid placement", ErrUnsupported)
		}
		out = loftface.Compose(out, loftface.Motion(section.mesh.xform))
	}
	return out, nil
}

func lfCapMap(section loftFaceSection, source *pointSectionRecord) (loftFaceMap, error) {
	if section.constructed != nil {
		return *section.constructed, nil
	}
	affine, err := lfSimilarity(section)
	if err != nil {
		return loftFaceMap{}, err
	}
	apex := loftface.Vec(source.apex)
	origin := loftface.Vec(source.plane.Origin)
	uAxis := loftface.Vec(source.plane.U)
	vAxis := loftface.Vec(source.plane.V)
	var n loftFaceRatVec
	var d *big.Rat
	if section.limit == nil {
		n = loftFaceRatVec{lfZero(), lfZero(), lfZero()}
		d = lfOne()
	} else {
		n = loftFaceRatVec{section.limit.nx, section.limit.ny, lfZero()}
		d = section.limit.d
	}
	g0, gu, gv := loftface.VDot(n, loftface.VSub(origin, apex)), loftface.VDot(n, uAxis), loftface.VDot(n, vAxis)
	if section.limit == nil {
		g0 = lfOne()
	}
	q0 := loftface.VScale(loftface.MatVec(affine.A, loftface.VSub(origin, apex)), d)
	qu := loftface.VScale(loftface.MatVec(affine.A, uAxis), d)
	qv := loftface.VScale(loftface.MatVec(affine.A, vAxis), d)
	stretch := loftface.AffineStretch(affine.A)
	if section.limit != nil {
		stretch = proofbound.ProductUpper(stretch, section.limit.mapStretch)
	}
	if proofbound.IsNonFinite(stretch) {
		return loftFaceMap{}, fmt.Errorf("%w: loft cap source stretch is not finite", ErrUnsupported)
	}
	return loftFaceMap{Constant: loftface.VAdd(loftface.MatVec(affine.A, apex), affine.B),
		Q0: q0, QU: qu, QV: qv, G0: g0, GU: gu, GV: gv, Stretch: stretch}, nil
}

func lfSourceBox(source *pointSectionRecord) ([4]*big.Rat, error) {
	var box [4]*big.Rat
	for i, v := range source.sourceBox {
		if proofbound.IsNonFinite(v) {
			return box,
				fmt.Errorf("%w: loft source rectangle is not finite", ErrUnsupported)
		}
		box[i] = lfF(v)
	}
	if box[0].Cmp(box[1]) >= 0 || box[2].Cmp(box[3]) >= 0 {
		return box, fmt.Errorf("%w: loft source rectangle has no area", ErrUnsupported)
	}
	return box, nil
}

type loftFaceEmbedding struct {
	maps        []loftFaceMap
	jacobianSum float64
	maximumNorm float64
	sign        int
}

type loftFaceCapBounds struct {
	stationRound        float64
	projectiveDeparture []float64
	polygonSpeed        []float64
	sourceSpeed         []float64
	sourceDeparture     []float64
}

func lfStationUV(point planePoint) (*big.Rat, *big.Rat) { return lfF(point.U), lfF(point.V) }

func lfCapBounds(ctx context.Context, capMap loftFaceMap, section loftFaceSection,
	source *pointSectionRecord) (loftFaceCapBounds, error) {
	n := len(source.sourceStation)
	var out loftFaceCapBounds
	if len(section.ring) != n || len(source.sourceArcUpper) != n || len(source.sourceMatched) != n {
		return out, fmt.Errorf("%w: loft source station proof is incomplete", ErrUnsupported)
	}
	out.projectiveDeparture = make([]float64, n)
	out.polygonSpeed = make([]float64, n)
	out.sourceSpeed = make([]float64, n)
	out.sourceDeparture = make([]float64, n)
	exact := make([]loftFaceRatVec, n)
	den := make([]*big.Rat, n)
	for j, station := range source.sourceStation {
		if j%256 == 0 {
			if err := ctx.Err(); err != nil {
				return out, err
			}
		}
		u, v := lfStationUV(station)
		den[j] = capMap.Denominator(u, v)
		if den[j].Sign() <= 0 {
			return out,
				fmt.Errorf("%w: loft station ray denominator is not positive", ErrUnsupported)
		}
		exact[j] = capMap.At(u, v)
		held := [3]float64{section.ring[j].X, section.ring[j].Y, section.ring[j].Z}
		perCoord := 0.0
		for axis := range held {
			perCoord = max(perCoord, proofbound.RatAbsDiff(exact[j][axis], held[axis]))
		}
		out.stationRound = max(out.stationRound, proofbound.Radius3D(perCoord))
	}
	for j := range n {
		jn := (j + 1) % n
		chord := loftface.VSub(exact[jn], exact[j])
		lengthL1 := lfAdd(lfAdd(lfAbs(chord[0]), lfAbs(chord[1])), lfAbs(chord[2]))
		minDen, maxDen := den[j], den[jn]
		if minDen.Cmp(maxDen) > 0 {
			minDen, maxDen = maxDen, minDen
		}
		shift := lfDiv(lfMul(lfAbs(lfSub(den[jn], den[j])), lengthL1),
			lfMul(big.NewRat(4, 1), minDen))
		speed := lfDiv(lfMul(maxDen, lengthL1), minDen)
		out.projectiveDeparture[j] = loftface.RatUp(shift)
		out.polygonSpeed[j] = loftface.RatUp(speed)
		out.sourceSpeed[j] = proofbound.ProductUpper(source.sourceArcUpper[j], capMap.Stretch)
		out.sourceDeparture[j] = proofbound.ProductUpper(
			max(source.sourceMatched[j], section.slice.base.meshBound), capMap.Stretch)
		if proofbound.IsNonFinite(out.projectiveDeparture[j]) ||
			proofbound.IsNonFinite(out.polygonSpeed[j]) ||
			proofbound.IsNonFinite(out.sourceSpeed[j]) ||
			proofbound.IsNonFinite(out.sourceDeparture[j]) {
			return out, fmt.Errorf("%w: loft cap cell has no finite source bound", ErrUnsupported)
		}
	}
	if proofbound.IsNonFinite(out.stationRound) {
		return out, fmt.Errorf("%w: loft cap station rounding is not finite", ErrUnsupported)
	}
	return out, nil
}

// proveLoftFaceBounds composes source-region, projective parameter, ruled
// wall, bilinear twist and held-station rounding bounds without replacing
// the authenticated fitted source by sampled cap edges.
func proveLoftFaceBounds(ctx context.Context, sections []loftFaceSection,
	source *pointSectionRecord, embedding loftFaceEmbedding,
	mesh facetedPayload) (facetedPayload, error) {
	proofs := make([]loftFaceCapBounds, len(sections))
	maxRound := 0.0
	for i, section := range sections {
		var err error
		proofs[i], err = lfCapBounds(ctx, embedding.maps[i], section, source)
		if err != nil {
			return facetedPayload{}, fmt.Errorf("loft cap %d: %w", i, err)
		}
		maxRound = max(maxRound, proofs[i].stationRound)
	}
	meshBound, wallLeg, twistVolume, trueWallArea := 0.0, 0.0, 0.0, 0.0
	n := len(source.sourceStation)
	for band := 0; band+1 < len(sections); band++ {
		lo, hi := proofs[band], proofs[band+1]
		for j := range n {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return facetedPayload{}, err
				}
			}
			jn := (j + 1) % n
			vLo, vHi := sections[band].ring[j], sections[band].ring[jn]
			wLo, wHi := sections[band+1].ring[j], sections[band+1].ring[jn]
			polygonMatch := max(proofbound.AbsSumUpper(lo.projectiveDeparture[j], lo.stationRound),
				proofbound.AbsSumUpper(hi.projectiveDeparture[j], hi.stationRound))
			wallArea := proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi,
				lo.polygonSpeed[j], hi.polygonSpeed[j], polygonMatch)
			wallLeg = proofbound.AbsSumUpper(wallLeg, proofbound.ProductUpper(polygonMatch, wallArea))
			twistVolume = proofbound.AbsSumUpper(twistVolume,
				proofbound.CellTwistVolumeAllow(vLo, vHi, wLo, wHi))
			smoothMatch := max(proofbound.AbsSumUpper(lo.sourceDeparture[j],
				lo.projectiveDeparture[j], lo.stationRound),
				proofbound.AbsSumUpper(hi.sourceDeparture[j],
					hi.projectiveDeparture[j], hi.stationRound))
			meshBound = max(meshBound, proofbound.AbsSumUpper(smoothMatch,
				proofbound.CellTwistOffsetUpper(vLo, vHi, wLo, wHi)))
			trueWallArea = proofbound.AbsSumUpper(trueWallArea,
				proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi,
					lo.sourceSpeed[j], hi.sourceSpeed[j], smoothMatch))
		}
	}
	for _, proof := range []loftFaceCapBounds{proofs[0], proofs[len(proofs)-1]} {
		for j := range proof.sourceDeparture {
			meshBound = max(meshBound, proofbound.AbsSumUpper(proof.sourceDeparture[j],
				proof.projectiveDeparture[j], proof.stationRound))
		}
	}
	// This whole-mesh sweep charges the exact source-map station coordinates
	// against the stored float vertices, including both end caps. The wall
	// matched-departure leg may also charge this rounding; duplication is safe.
	roundArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, maxRound)
	if err != nil {
		return facetedPayload{}, err
	}
	roundSweep := proofbound.SweptVolumeAllow(maxRound, roundArea)
	sourceRegion := proofbound.ProductUpper(source.sourceAreaDelta, embedding.jacobianSum)
	volumeGap := proofbound.AbsSumUpper(sourceRegion, wallLeg, twistVolume, roundSweep)
	base := sections[0].slice.base
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.Verts, base.Tris, base.meshBound)
	if err != nil {
		return facetedPayload{}, err
	}
	baseArea = proofbound.AbsSumUpper(baseArea, base.areaSlack)
	endCapArea := 0.0
	for _, i := range []int{0, len(sections) - 1} {
		stretch2 := proofbound.ProductUpper(embedding.maps[i].Stretch, embedding.maps[i].Stretch)
		endCapArea = proofbound.AbsSumUpper(endCapArea, proofbound.ProductUpper(baseArea, stretch2))
	}
	heldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, 0)
	if err != nil {
		return facetedPayload{}, err
	}
	areaSlack := proofbound.AbsSumUpper(trueWallArea, endCapArea, heldArea)
	if proofbound.IsNonFinite(meshBound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) {
		return facetedPayload{}, fmt.Errorf("%w: face-fed loft proof has no finite bounds", ErrUnsupported)
	}
	mesh.vertexBound = make([]float64, len(mesh.Verts))
	for i := range mesh.vertexBound {
		mesh.vertexBound[i] = meshBound
	}
	mesh.meshBound = meshBound
	mesh.volSymDiff = volumeGap
	mesh.areaSlack = areaSlack
	mesh.dPair = facetedSourceDiameterUpper(mesh.Verts, meshBound)
	mesh.contactAudited = true
	return mesh, nil
}

func proveLoftFaceEmbedding(ctx context.Context, sections []loftFaceSection,
	source *pointSectionRecord) (loftFaceEmbedding, error) {
	var proof loftFaceEmbedding
	box, err := lfSourceBox(source)
	if err != nil {
		return proof, err
	}
	proof.maps = make([]loftFaceMap, len(sections))
	for i, section := range sections {
		if err := ctx.Err(); err != nil {
			return proof, err
		}
		proof.maps[i], err = lfCapMap(section, source)
		if err != nil {
			return proof, fmt.Errorf("loft section %d: %w", i, err)
		}
		if err := loftface.PositiveDenominator(proof.maps[i], box); err != nil {
			return proof, fmt.Errorf("loft section %d: %w", i, err)
		}
	}
	if err := loftface.PlaneOrder(proof.maps, box); err != nil {
		return proof, err
	}
	for band := 0; band+1 < len(proof.maps); band++ {
		if err := ctx.Err(); err != nil {
			return proof, err
		}
		rho, jac, sign, gateErr := loftface.BandGate(ctx, proof.maps[band], proof.maps[band+1], box)
		if gateErr != nil {
			return proof, fmt.Errorf("loft band %d: %w", band, gateErr)
		}
		if proof.sign != 0 && proof.sign != sign {
			return proof, fmt.Errorf("%w: loft bands reverse their source orientation", ErrUnsupported)
		}
		proof.sign = sign
		proof.maximumNorm = max(proof.maximumNorm, rho)
		proof.jacobianSum = proofbound.AbsSumUpper(proof.jacobianSum, jac)
	}
	return proof, nil
}

func orientLoftFaces(mesh *facetedPayload, base facetedPayload,
	source *pointSectionRecord, jacobianSign int) error {
	n := len(source.sourceStation)
	if n < 3 || len(base.Tris) <= n || jacobianSign == 0 {
		return fmt.Errorf("%w: loft source cap has no orientation", ErrUnsupported)
	}
	polygonSign := 0
	for _, tri := range base.Tris[n:] {
		if tri[0] <= 0 || tri[1] <= 0 || tri[2] <= 0 ||
			tri[0] > n || tri[1] > n || tri[2] > n {
			return fmt.Errorf("%w: loft source cap has invalid station indices", ErrUnsupported)
		}
		a, b, c := source.sourceStation[tri[0]-1], source.sourceStation[tri[1]-1],
			source.sourceStation[tri[2]-1]
		cross := lfSub(lfMul(lfSub(lfF(b.U), lfF(a.U)), lfSub(lfF(c.V), lfF(a.V))),
			lfMul(lfSub(lfF(b.V), lfF(a.V)), lfSub(lfF(c.U), lfF(a.U))))
		sign := cross.Sign()
		if sign == 0 || (polygonSign != 0 && sign != polygonSign) {
			return fmt.Errorf("%w: loft source cap triangulation changes orientation", ErrUnsupported)
		}
		polygonSign = sign
	}
	if polygonSign != jacobianSign {
		for i := range mesh.Tris {
			mesh.Tris[i][1], mesh.Tris[i][2] = mesh.Tris[i][2], mesh.Tris[i][1]
		}
		mesh.groups[len(mesh.groups)-2].reversed = false
		mesh.groups[len(mesh.groups)-1].reversed = true
	}
	return nil
}
