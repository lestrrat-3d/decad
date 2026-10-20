package prismpattern

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// UnionMany admits only an authenticated circular prism and
// rigid copies of one extruded section. Each later proof stage may refuse;
// the public Union commits only a fully audited result.
func UnionMany(ctx context.Context, env topo.Env, mesher verifiedmesh.Mesher,
	operands []*topo.Body) (*topo.Body, bool, error) {
	source, matched, err := SourceOf(ctx, operands)
	if !matched || err != nil {
		return nil, matched, err
	}
	blank := source.Blank
	if blank.SurfaceResult || blank.SectionDelta != 0 || blank.Z0Delta != 0 || blank.Z1Delta != 0 ||
		blank.Frame.Origin() != (r3.Vec{}) || blank.Frame.U() != (r3.Vec{X: 1}) ||
		blank.Frame.V() != (r3.Vec{Y: 1}) || blank.Z0 != 0 || blank.Z1 <= 0 ||
		source.RootIndex != len(source.Section.Profile.Outer.Segments)-1 ||
		source.RootArc.TStart != 1 || source.RootArc.TEnd != 0 {
		return nil, true, fmt.Errorf("%w: copied-prism circular join needs a positive axial blank and trailing inward root arc",
			decaderr.ErrUnsupported)
	}
	parts := make([]facetedrecord.PrismPatternPart, len(source.Sections))
	for i, body := range source.Sections {
		if err := ctx.Err(); err != nil {
			return nil, true, err
		}
		p, ok := body.Payload.(prismrecord.Payload)
		if !ok {
			return nil, true, fmt.Errorf("%w: copied-prism section %d lost its prism payload", decaderr.ErrUnsupported, i+1)
		}
		if p.SurfaceResult || p.SectionDelta != 0 || p.Z0Delta != 0 || p.Z1Delta != 0 {
			return nil, true, fmt.Errorf("%w: copied-prism section %d has a displaced or open source", decaderr.ErrUnsupported, i+1)
		}
		m, err := mesher.VerifyAll(ctx, body, 0.02)
		if err != nil {
			return nil, true, err
		}
		if !m.BoundaryOK || !m.SymDiffOK {
			return nil, true, fmt.Errorf("%w: copied-prism section %d lacks a complete mesh proof", decaderr.ErrUnsupported, i+1)
		}
		rootStart, err := meshRoot(m, body, source.RootIndex, blank.Z0, blank.Z1)
		if err != nil {
			return nil, true, err
		}
		first := m.Vertices[0]
		last := m.Vertices[2*rootStart]
		start, end := math.Atan2(first.Y, first.X), math.Atan2(last.Y, last.X)
		startIV := proofbound.Atan2Interval(proofarith.FloatRat(first.Y), proofarith.FloatRat(first.X), false)
		endIV := proofbound.Atan2Interval(proofarith.FloatRat(last.Y), proofarith.FloatRat(last.X), false)
		for end <= start {
			end += 2 * math.Pi
			endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
		}
		if end-start <= 0 || end-start >= math.Pi/2 ||
			startIV.Hi.Cmp(endIV.Lo) >= 0 {
			return nil, true, fmt.Errorf("%w: copied-prism section %d has no ordered root sector", decaderr.ErrUnsupported, i+1)
		}
		parts[i] = facetedrecord.PrismPatternPart{Body: body, Prism: p, Mesh: m, RootStart: rootStart,
			Start: start, End: end, StartIV: startIV, EndIV: endIV}
	}
	sort.Slice(parts, func(i, j int) bool { return parts[i].Start < parts[j].Start })
	for i := range parts {
		for j := i + 1; j < len(parts); j++ {
			if !MeshSeparated(parts[i], parts[j]) {
				return nil, true, fmt.Errorf("%w: copied-prism sections %d and %d have no certified separating plane",
					decaderr.ErrUnsupported, i+1, j+1)
			}
		}
	}
	for i := range parts {
		a, b := &parts[i], &parts[(i+1)%len(parts)]
		nextStart := b.Start
		nextIV := b.StartIV
		for nextStart <= a.End {
			nextStart += 2 * math.Pi
			nextIV = proofbound.IntervalAdd(nextIV, proofbound.TwoPiInterval())
		}
		if nextStart-a.End >= 2*math.Pi || a.EndIV.Hi.Cmp(nextIV.Lo) >= 0 {
			return nil, true, fmt.Errorf("%w: copied-prism root sectors have no ordered circular gap", decaderr.ErrUnsupported)
		}
	}
	mesh, outlines, groups, step, round, err := Shell(ctx, operands[0], source, parts)
	if err != nil {
		return nil, true, err
	}
	body, err := joinedBody(ctx, env, operands[0], source, parts,
		mesh, outlines, groups, step, round)
	return body, true, err
}

func meshRoot(m *topo.Mesh, body *topo.Body, rootIndex int, z0, z1 float64) (int, error) {
	role := fmt.Sprintf("side(0,%d)", rootIndex)
	var roots []*topo.Face
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin.Role == role {
				roots = append(roots, face)
			}
		}
	}
	return tessellation.PrismPatternMeshRoot(m.Vertices, len(m.Triangles), m.Source, roots, z0, z1)
}

// joinedBody charges the only changed source boundary to the
// circular root band. The authenticated source root arcs lie strictly inside
// the blank. Each straight connector crosses its circle once, and every
// other source wall stays outside it. Thus the true and held exteriors agree
// outside the connector reach and ring-chord band, apart from each source
// mesh's already certified boundary and occupied-volume error.
func joinedBody(ctx context.Context, env topo.Env,
	blank *topo.Body, source *facetedrecord.PrismPatternSource, parts []facetedrecord.PrismPatternPart,
	mesh *facetedrecord.JoinedMesh, outlines [2][]int, groups [3]int, maxStep, ringRound float64) (*topo.Body, error) {
	radius := source.BlankRadius
	toothBound, toothVolume, toothArea := 0.0, 0.0, 0.0
	connectorReach, rootHeldExcess := 0.0, 0.0
	for i := range parts {
		part := &parts[i]
		toothBound = max(toothBound, units.Millimeters(part.Mesh.Bound).Base())
		toothVolume = proofbound.AbsSumUpper(toothVolume, part.Mesh.VolSymDiff)
		area := part.Body.Area
		toothArea = proofbound.AbsSumUpper(toothArea, area.Value.Base(), area.Bound.Base())
		reach, err := ConnectorReach(part.Prism, source.RootArc, radius,
			source.RootIndex)

		if err != nil {
			return nil, err
		}
		connectorReach = max(connectorReach, reach)
		n := len(part.Mesh.Vertices) / 2
		for j := range n {
			if j < part.RootStart {
				continue
			}
			v := part.Mesh.Vertices[2*j]
			r2 := RadialSq(sectionrecord.Point2{U: v.X, V: v.Y})
			excess := new(big.Rat).Sub(proofarith.FloatRat(proofbound.RatSqrtUp(r2)),
				proofarith.FloatRat(radius))
			if excess.Sign() > 0 {
				rootHeldExcess = max(rootHeldExcess, proofbound.RatFloatUp(excess))
			}
		}
		// The retained fitted and tip facets stay strictly outside the blank
		// even after their source-to-held departure. The first and last
		// connector walls are the only retained pieces that enter the band.
		for j := 1; j < part.RootStart-1; j++ {
			for side := range 2 {
				tri := part.Mesh.Triangles[2*j+side]
				minimum := loftcylinder.TriangleMinRadius(part.Mesh.Vertices, tri)
				if minimum <= proofbound.AbsSumUpper(radius, units.Millimeters(part.Mesh.Bound).Base()) {
					return nil, fmt.Errorf("%w: copied-prism exterior wall %d lacks circular clearance",
						decaderr.ErrUnsupported, j)
				}
			}
		}
	}
	bound, volumeGap, err := UnionProofBounds(source, radius, toothBound, toothVolume,
		connectorReach, rootHeldExcess, maxStep, ringRound)
	if err != nil {
		return nil, err
	}
	areaUpper, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return nil, err
	}
	blankArea := blank.Area
	areaSlack := proofbound.AbsSumUpper(areaUpper,
		blankArea.Value.Base(), blankArea.Bound.Base(), toothArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: copied-prism join has no finite occupied-volume proof", decaderr.ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	fp := facetedrecord.Payload{Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src, Groups: mesh.Groups,
		AxialSectionJoin: true,
		PrismPatternJoin: &facetedrecord.PrismPatternJoinRecord{Blank: blank, Source: source, Parts: parts,
			Outlines: outlines, Groups: groups, MaxStep: maxStep, RingRound: ringRound,
			Mesh: mesh},
		VertexBound: vertexBound, ContactAudited: true,
		MeshBound: bound, VolSymDiff: volumeGap, AreaSlack: areaSlack,
		DPair: facetedrecord.SourceDiameterUpper(mesh.Verts, bound), Xform: r3.Identity()}
	if proofbound.IsNonFinite(fp.DPair) {
		return nil, fmt.Errorf("%w: copied-prism join has no finite diameter proof", decaderr.ErrUnsupported)
	}
	if err := loftcylinder.TagCaps(&fp, blank, [2]int{groups[1], groups[2]},
		[2]float64{source.Blank.Z0, source.Blank.Z1}, env.Ref); err != nil {
		return nil, err
	}
	return facetedrecord.Build(ctx, env, fp)
}
