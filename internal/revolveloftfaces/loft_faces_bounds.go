package revolveloftfaces

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// BoundsSection carries only the held ring and source-base bound used by cap bounds.
type BoundsSection struct {
	Ring          []r3.Vec
	BaseMeshBound float64
}

func lfDiv(a, b *big.Rat) *big.Rat { return proofarith.QuoRat(new(big.Rat), a, b) }
func lfAbs(a *big.Rat) *big.Rat    { return new(big.Rat).Abs(a) }

func CapBounds(ctx context.Context, capMap loftface.Map, section BoundsSection,
	source *facetedrecord.PointSectionRecord) (facetedrecord.LoftFaceCapBounds, error) {
	n := len(source.SourceStation)
	var out facetedrecord.LoftFaceCapBounds
	if len(section.Ring) != n || len(source.SourceArcUpper) != n || len(source.SourceMatched) != n {
		return out, fmt.Errorf("%w: loft source station proof is incomplete", decaderr.ErrUnsupported)
	}
	out.ProjectiveDeparture = make([]float64, n)
	out.PolygonSpeed = make([]float64, n)
	out.SourceSpeed = make([]float64, n)
	out.SourceDeparture = make([]float64, n)
	exact := make([]loftface.RatVec, n)
	den := make([]*big.Rat, n)
	for j, station := range source.SourceStation {
		if j%256 == 0 {
			if err := ctx.Err(); err != nil {
				return out, err
			}
		}
		u, v := lfF(station.U), lfF(station.V)
		den[j] = capMap.Denominator(u, v)
		if den[j].Sign() <= 0 {
			return out,
				fmt.Errorf("%w: loft station ray denominator is not positive", decaderr.ErrUnsupported)
		}
		exact[j] = capMap.At(u, v)
		held := [3]float64{section.Ring[j].X, section.Ring[j].Y, section.Ring[j].Z}
		perCoord := 0.0
		for axis := range held {
			perCoord = max(perCoord, proofbound.RatAbsDiff(exact[j][axis], held[axis]))
		}
		out.StationRound = max(out.StationRound, proofbound.Radius3D(perCoord))
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
		out.ProjectiveDeparture[j] = loftface.RatUp(shift)
		out.PolygonSpeed[j] = loftface.RatUp(speed)
		out.SourceSpeed[j] = proofbound.ProductUpper(source.SourceArcUpper[j], capMap.Stretch)
		out.SourceDeparture[j] = proofbound.ProductUpper(
			max(source.SourceMatched[j], section.BaseMeshBound), capMap.Stretch)
		if proofbound.IsNonFinite(out.ProjectiveDeparture[j]) ||
			proofbound.IsNonFinite(out.PolygonSpeed[j]) ||
			proofbound.IsNonFinite(out.SourceSpeed[j]) ||
			proofbound.IsNonFinite(out.SourceDeparture[j]) {
			return out, fmt.Errorf("%w: loft cap cell has no finite source bound", decaderr.ErrUnsupported)
		}
	}
	if proofbound.IsNonFinite(out.StationRound) {
		return out, fmt.Errorf("%w: loft cap station rounding is not finite", decaderr.ErrUnsupported)
	}
	return out, nil
}

// ProveBounds composes source-region, projective parameter, ruled
// wall, bilinear twist and held-station rounding bounds without replacing
// the authenticated fitted source by sampled cap edges.
func ProveBounds(ctx context.Context, sections []BoundsSection,
	source *facetedrecord.PointSectionRecord, maps []loftface.Map, jacobianSum float64, mesh facetedrecord.Payload,
	base facetedrecord.Payload) (facetedrecord.Payload, error) {
	proofs := make([]facetedrecord.LoftFaceCapBounds, len(sections))
	maxRound := 0.0
	for i, section := range sections {
		var err error
		proofs[i], err = CapBounds(ctx, maps[i], section, source)
		if err != nil {
			return facetedrecord.Payload{}, fmt.Errorf("loft cap %d: %w", i, err)
		}
		maxRound = max(maxRound, proofs[i].StationRound)
	}
	meshBound, wallLeg, twistVolume, trueWallArea := 0.0, 0.0, 0.0, 0.0
	n := len(source.SourceStation)
	for band := 0; band+1 < len(sections); band++ {
		lo, hi := proofs[band], proofs[band+1]
		for j := range n {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return facetedrecord.Payload{}, err
				}
			}
			jn := (j + 1) % n
			vLo, vHi := sections[band].Ring[j], sections[band].Ring[jn]
			wLo, wHi := sections[band+1].Ring[j], sections[band+1].Ring[jn]
			polygonMatch := max(proofbound.AbsSumUpper(lo.ProjectiveDeparture[j], lo.StationRound),
				proofbound.AbsSumUpper(hi.ProjectiveDeparture[j], hi.StationRound))
			wallArea := proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi,
				lo.PolygonSpeed[j], hi.PolygonSpeed[j], polygonMatch)
			wallLeg = proofbound.AbsSumUpper(wallLeg, proofbound.ProductUpper(polygonMatch, wallArea))
			twistVolume = proofbound.AbsSumUpper(twistVolume,
				proofbound.CellTwistVolumeAllow(vLo, vHi, wLo, wHi))
			smoothMatch := max(proofbound.AbsSumUpper(lo.SourceDeparture[j],
				lo.ProjectiveDeparture[j], lo.StationRound),
				proofbound.AbsSumUpper(hi.SourceDeparture[j],
					hi.ProjectiveDeparture[j], hi.StationRound))
			meshBound = max(meshBound, proofbound.AbsSumUpper(smoothMatch,
				proofbound.CellTwistOffsetUpper(vLo, vHi, wLo, wHi)))
			trueWallArea = proofbound.AbsSumUpper(trueWallArea,
				proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi,
					lo.SourceSpeed[j], hi.SourceSpeed[j], smoothMatch))
		}
	}
	for _, proof := range []facetedrecord.LoftFaceCapBounds{proofs[0], proofs[len(proofs)-1]} {
		for j := range proof.SourceDeparture {
			meshBound = max(meshBound, proofbound.AbsSumUpper(proof.SourceDeparture[j],
				proof.ProjectiveDeparture[j], proof.StationRound))
		}
	}
	// This whole-mesh sweep charges the exact source-map station coordinates
	// against the stored float vertices, including both end caps. The wall
	// matched-departure leg may also charge this rounding; duplication is safe.
	roundArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, maxRound)
	if err != nil {
		return facetedrecord.Payload{}, err
	}
	roundSweep := proofbound.SweptVolumeAllow(maxRound, roundArea)
	sourceRegion := proofbound.ProductUpper(source.SourceAreaDelta, jacobianSum)
	volumeGap := proofbound.AbsSumUpper(sourceRegion, wallLeg, twistVolume, roundSweep)
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.Verts, base.Tris, base.MeshBound)
	if err != nil {
		return facetedrecord.Payload{}, err
	}
	baseArea = proofbound.AbsSumUpper(baseArea, base.AreaSlack)
	endCapArea := 0.0
	for _, i := range []int{0, len(sections) - 1} {
		stretch2 := proofbound.ProductUpper(maps[i].Stretch, maps[i].Stretch)
		endCapArea = proofbound.AbsSumUpper(endCapArea, proofbound.ProductUpper(baseArea, stretch2))
	}
	heldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, 0)
	if err != nil {
		return facetedrecord.Payload{}, err
	}
	areaSlack := proofbound.AbsSumUpper(trueWallArea, endCapArea, heldArea)
	if proofbound.IsNonFinite(meshBound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) {
		return facetedrecord.Payload{}, fmt.Errorf("%w: face-fed loft proof has no finite bounds", decaderr.ErrUnsupported)
	}
	mesh.VertexBound = make([]float64, len(mesh.Verts))
	for i := range mesh.VertexBound {
		mesh.VertexBound[i] = meshBound
	}
	mesh.MeshBound = meshBound
	mesh.VolSymDiff = volumeGap
	mesh.AreaSlack = areaSlack
	mesh.DPair = facetedrecord.SourceDiameterUpper(mesh.Verts, meshBound)
	mesh.ContactAudited = true
	return mesh, nil
}
