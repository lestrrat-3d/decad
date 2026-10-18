package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// The original operands remain the exact source of the result. The paired
// proof teeth have the same XY sections and source identity, with an upward
// charge for the one-ulp axial placement defect in their Loft proofs.
type ChamferBridgeSourcesInput struct {
	ProofTeeth, OriginalTeeth []*topo.Body
	AxialBridge               []float64
	Z0, Z1, BridgeBound       float64
}

func ChamferBridgeSources(source *ChamferBridgeSourcesInput) (float64, *big.Rat, error) {
	if source == nil || len(source.ProofTeeth) == 0 || len(source.ProofTeeth) != len(source.OriginalTeeth) ||
		len(source.ProofTeeth) != len(source.AxialBridge) || source.Z1 <= source.Z0 ||
		proofbound.IsNonFinite(source.Z0) || proofbound.IsNonFinite(source.Z1) {
		return 0, nil, fmt.Errorf("%w: cap chamfer lost its original and proof teeth", decaderr.ErrUnsupported)
	}
	minTop := proofarith.FloatRat(source.Z1)
	bridgeBound := source.BridgeBound
	var bridgedTop *big.Rat
	for i, proofBody := range source.ProofTeeth {
		originalBody := source.OriginalTeeth[i]
		if originalBody == nil || proofBody == nil {
			return 0, nil, fmt.Errorf("%w: cap chamfer has an absent tooth", decaderr.ErrUnsupported)
		}
		original, okOriginal := originalBody.Payload.(loftrecord.Payload)
		proxy, okProxy := proofBody.Payload.(loftrecord.Payload)
		shift := source.AxialBridge[i]
		if !okOriginal || !okProxy || !original.AuthenticatedReconstruction ||
			!proxy.AuthenticatedReconstruction || original.SourceIdentity == nil ||
			original.SourceIdentity != proxy.SourceIdentity ||
			len(original.Verts) != len(proxy.Verts) || len(original.Verts)%2 != 0 ||
			len(original.Verts) < 6 || !slices.Equal(original.Tris, proxy.Tris) ||
			len(original.Segment) != len(proxy.Segment) || shift < 0 ||
			proofbound.IsNonFinite(shift) ||
			proxy.Proof.FacetDeparture < original.Proof.FacetDeparture ||
			!AxialPlacement(proxy.Xform) {
			return 0, nil, fmt.Errorf("%w: cap chamfer tooth bridge lost its source identity", decaderr.ErrUnsupported)
		}
		for j := range original.Segment {
			if !slices.Equal(original.Segment[j], proxy.Segment[j]) {
				return 0, nil, fmt.Errorf("%w: cap chamfer tooth bridge changed its wall cells", decaderr.ErrUnsupported)
			}
		}
		n := len(original.Verts) / 2
		for j, p := range original.Verts {
			q := proxy.Verts[j]
			if p.X != q.X || p.Y != q.Y ||
				j < n && (p.Z != source.Z0 || q.Z != source.Z0) ||
				j >= n && q.Z != source.Z1 {
				return 0, nil, fmt.Errorf("%w: cap chamfer bridge changed a cap station", decaderr.ErrUnsupported)
			}
			if j >= n {
				vertexGap := new(big.Rat).Sub(proofarith.FloatRat(source.Z1), proofarith.FloatRat(p.Z))
				if vertexGap.Sign() < 0 || vertexGap.Cmp(proofarith.FloatRat(shift)) > 0 {
					return 0, nil, fmt.Errorf("%w: cap chamfer top station exceeds its bridge", decaderr.ErrUnsupported)
				}
				if top := proofarith.FloatRat(p.Z); top.Cmp(minTop) < 0 {
					minTop = top
				}
			}
		}
		basis := original.Xform.Basis()
		if basis.EX != proxy.Xform.Basis().EX || basis.EY != proxy.Xform.Basis().EY ||
			basis.EZ.X != 0 || basis.EZ.Y != 0 || basis.EZ.Z <= 0 || basis.EZ.Z > 1 ||
			basis.EZ.Z != 1 && basis.EZ.Z != math.Nextafter(1, 0) ||
			original.Xform.Translation() != proxy.Xform.Translation() ||
			proxy.Xform.Basis().EZ != (r3.Vec{Z: 1}) {
			return 0, nil, fmt.Errorf("%w: cap chamfer bridge is not one axial scale", decaderr.ErrUnsupported)
		}
		if basis.EZ.Z != 1 && source.Z0 != 0 {
			return 0, nil, fmt.Errorf("%w: cap chamfer bridge has a displaced lower cap", decaderr.ErrUnsupported)
		}
		analyticTop := new(big.Rat).Mul(proofarith.FloatRat(basis.EZ.Z), proofarith.FloatRat(source.Z1))
		analyticGap := new(big.Rat).Sub(proofarith.FloatRat(source.Z1), analyticTop)
		if analyticGap.Sign() < 0 || analyticGap.Cmp(proofarith.FloatRat(shift)) > 0 {
			return 0, nil, fmt.Errorf("%w: cap chamfer analytic cap exceeds its bridge", decaderr.ErrUnsupported)
		}
		if analyticTop.Cmp(minTop) < 0 {
			minTop = analyticTop
		}
		if shift > 0 {
			if bridgedTop != nil && analyticTop.Cmp(bridgedTop) != 0 {
				return 0, nil, fmt.Errorf("%w: cap chamfer bridge has different source cap heights", decaderr.ErrUnsupported)
			}
			bridgedTop = analyticTop
		}
		bridgeBound = max(bridgeBound, shift)
	}
	return bridgeBound, minTop, nil
}

// A strict section of the top band at the lowest original tooth cap is one
// simple inset-adjacent loop. Every intervening section stays in the same
// annular band: all band facets span only two heights, and the earlier exact
// embedding audit excludes pinches and crossings. A source cap at any height
// in this slab therefore differs from the held inset by at most epsilon in Z
// and twice the matched section motion in XY. The second motion covers points
// in the thin swept rim neighborhood, including the source root slit.
func ChamferBridgeCap(ctx context.Context, fp facetedrecord.Payload,
	mesh *facetedrecord.JoinedMesh, ideal []proofarith.Xpt,
	band [2]int, inset []int, setback, z1, bridgeBound float64,
	topSourceLevel *big.Rat, outerTop []int) (float64, error) {
	if bridgeBound == 0 {
		return 0, nil
	}
	if topSourceLevel == nil || band[0] < 0 || band[1] > len(mesh.Tris) ||
		band[0] >= band[1] || len(ideal) != len(mesh.Verts) || len(inset) < 3 {
		return 0, fmt.Errorf("%w: cap chamfer bridge has no ideal top band", decaderr.ErrUnsupported)
	}
	capLevel := proofarith.FloatRat(z1)
	sideLevel := new(big.Rat).Sub(capLevel, proofarith.FloatRat(setback))
	level := topSourceLevel
	if level.Cmp(sideLevel) <= 0 || level.Cmp(capLevel) >= 0 {
		return 0, fmt.Errorf("%w: cap chamfer bridge leaves its top band", decaderr.ErrUnsupported)
	}
	section := make([]proofarith.Xpt, 0, band[1]-band[0])
	lookup := make(map[string]int, band[1]-band[0])
	adjacent := make([][]int, 0, band[1]-band[0])
	intern := func(p proofarith.Xpt) int {
		if index, ok := lookup[p.Key()]; ok {
			return index
		}
		index := len(section)
		lookup[p.Key()] = index
		section = append(section, p)
		adjacent = append(adjacent, nil)
		return index
	}
	maxSpan2 := new(big.Rat)
	for i := band[0]; i < band[1]; i++ {
		if i%64 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, err
			}
		}
		tri := mesh.Tris[i]
		var crossing []int
		for k := range 3 {
			a, b := ideal[tri[k]], ideal[tri[(k+1)%3]]
			az, bz := meshbool.RatCoordOf(a, 2), meshbool.RatCoordOf(b, 2)
			if az.Cmp(bz) == 0 {
				continue
			}
			if az.Cmp(level)*bz.Cmp(level) >= 0 {
				return 0, fmt.Errorf("%w: top band edge does not span source cap", decaderr.ErrUnsupported)
			}
			t := new(big.Rat).Quo(new(big.Rat).Sub(level, az), new(big.Rat).Sub(bz, az))
			crossing = append(crossing, intern(proofarith.Xlerp(a, b, new(big.Int).Set(t.Num()), new(big.Int).Set(t.Denom()))))
			dx := new(big.Rat).Sub(meshbool.RatCoordOf(a, 0), meshbool.RatCoordOf(b, 0))
			dy := new(big.Rat).Sub(meshbool.RatCoordOf(a, 1), meshbool.RatCoordOf(b, 1))
			span2 := new(big.Rat).Add(new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy))
			if span2.Cmp(maxSpan2) > 0 {
				maxSpan2 = span2
			}
		}
		if len(crossing) != 2 || crossing[0] == crossing[1] {
			return 0, fmt.Errorf("%w: top band facet has no transverse source-cap section", decaderr.ErrUnsupported)
		}
		a, b := crossing[0], crossing[1]
		adjacent[a] = append(adjacent[a], b)
		adjacent[b] = append(adjacent[b], a)
	}
	for _, neighbors := range adjacent {
		if len(neighbors) != 2 {
			return 0, fmt.Errorf("%w: source-cap section is not one loop", decaderr.ErrUnsupported)
		}
	}
	ring := make([]int, 0, len(section))
	previous, current := -1, 0
	for {
		if len(ring) >= len(section) {
			return 0, fmt.Errorf("%w: source-cap section repeats a vertex", decaderr.ErrUnsupported)
		}
		ring = append(ring, current)
		next := adjacent[current][0]
		if next == previous {
			next = adjacent[current][1]
		}
		previous, current = current, next
		if current == 0 {
			break
		}
	}
	if len(ring) != len(section) {
		return 0, fmt.Errorf("%w: source-cap section has another component", decaderr.ErrUnsupported)
	}
	xverts := append(append([]proofarith.Xpt(nil), ideal...), section...)
	sectionRing := make([]int, len(ring))
	for i, at := range ring {
		sectionRing[i] = len(ideal) + at
	}
	area2 := new(big.Rat)
	for i, at := range sectionRing {
		a, b := xverts[at], xverts[sectionRing[(i+1)%len(sectionRing)]]
		area2.Add(area2, new(big.Rat).Sub(
			new(big.Rat).Mul(meshbool.RatCoordOf(a, 0), meshbool.RatCoordOf(b, 1)),
			new(big.Rat).Mul(meshbool.RatCoordOf(a, 1), meshbool.RatCoordOf(b, 0))))
	}
	if area2.Sign() < 0 {
		for i, j := 0, len(sectionRing)-1; i < j; i, j = i+1, j-1 {
			sectionRing[i], sectionRing[j] = sectionRing[j], sectionRing[i]
		}
	}
	if !ChamferExactRingSimple(ctx, xverts, sectionRing) {
		return 0, fmt.Errorf("%w: source-cap section self-intersects", decaderr.ErrUnsupported)
	}
	nested, err := ChamferExactNested(ctx, xverts, sectionRing, inset)
	if err != nil {
		return 0, err
	}
	if !nested {
		return 0, fmt.Errorf("%w: source-cap section does not enclose the inset", decaderr.ErrUnsupported)
	}
	epsilon := proofbound.RatFloatUp(new(big.Rat).Sub(capLevel, level))
	span := proofbound.RatSqrtUp(maxSpan2)
	lateral := proofbound.ProductUpper(proofbound.RatFloatUp(new(big.Rat).Quo(
		new(big.Rat).Sub(capLevel, level), proofarith.FloatRat(setback))), span)
	gap := proofbound.AbsSumUpper(fp.MeshBound, lateral)
	if err := ChamferHeldOutlineGap(ctx, mesh.Verts, outerTop, inset, gap); err != nil {
		return 0, err
	}
	reach := proofbound.AbsSumUpper(epsilon, lateral, lateral)
	if reach <= 0 || proofbound.IsNonFinite(reach) {
		return 0, fmt.Errorf("%w: cap chamfer bridge has no finite source reach", decaderr.ErrUnsupported)
	}
	return reach, nil
}
