package retainedblend

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/chaintol"
	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// The saddle chamfer's tool (agent-docs/retained-edge-blend-design.md §4.9).
// In the fixed pose of agent-docs/cross-cylinder-bore-chamfer-design.md (shaft
// axis Z, bore axis X, the axes meeting at the origin, the mouth at +X), each
// of the crossbore.ToolStations stations θ_j holds four vertices:
//
//   - O'', the outer foot extended past the shaft wall along the band;
//   - A, O'' pushed out radially from the shaft's axis to R + r;
//   - K, the point (X + r, r·cos θ/2, r·sin θ/2) in the bore's extension
//     outside the shaft, with X = √(R² − r²cos²θ);
//   - B'', the bore foot extended into the bore along the band.
//
// The tool is the ring of quads joining consecutive stations over the four
// sides O''→A→K→B''→O''. The side B''→O'' is the chamfer band; the other three
// lie outside the true solid by a proven margin.

const (
	toolBand = iota
	toolSide
)

// saddleTool is the tool mesh in world coordinates, with each vertex's bound.
type saddleTool struct {
	verts []r3.Vec
	beta  []float64
	tris  [][3]int
	src   []int
	proof crossbore.ToolProof
	// bandArea is the held float area of the band's facets, within bandSlop
	// of their exact area.
	bandArea, bandSlop float64
}

// stationPoints returns the four local vertices of station j.
func stationPoints(R, r, d float64, j int) [4]r3.Vec {
	o, b := crossbore.BandEnds(R, r, d, j)
	theta := 2 * math.Pi * float64(j) / crossbore.ToolStations
	s, c := math.Sincos(theta)
	x := math.Sqrt(R*R - r*r*c*c)
	k := r3.Vec{X: x + r, Y: r * c / 2, Z: r * s / 2}
	scale := (R + r) / math.Hypot(o.X, o.Y)
	a := r3.Vec{X: o.X * scale, Y: o.Y * scale, Z: o.Z}
	return [4]r3.Vec{o, a, k, b}
}

// buildSaddleTool places the stations, joins them and proves the band.
func buildSaddleTool(ctx context.Context, s saddle, d float64) (*saddleTool, error) {
	const n = crossbore.ToolStations
	t := &saddleTool{verts: make([]r3.Vec, 0, 4*n)}
	for j := range n {
		for _, p := range stationPoints(s.key.R, s.key.r, d, j) {
			t.verts = append(t.verts, s.key.pose.World(p))
		}
	}
	for _, v := range t.verts {
		if !proofbound.FiniteVec(v) {
			return nil, fmt.Errorf("%w: the saddle chamfer's tool has no finite vertex", decaderr.ErrUnsupported)
		}
	}
	for j := range n {
		next := (j + 1) % n
		for side := range 4 {
			a, b := 4*j+side, 4*j+(side+1)%4
			c, e := 4*next+(side+1)%4, 4*next+side
			src := toolSide
			if side == 3 {
				src = toolBand
			}
			t.tris = append(t.tris, [3]int{a, b, c}, [3]int{a, c, e})
			t.src = append(t.src, src, src)
		}
	}
	// Wind the ring outward: a negative signed volume flips every facet. The
	// closed-shell audit of the tool's build decides the winding is consistent.
	volume := 0.0
	for _, f := range t.tris {
		a, b, c := t.verts[f[0]], t.verts[f[1]], t.verts[f[2]]
		volume += a.Dot(b.Cross(c))
	}
	if volume < 0 {
		for i := range t.tris {
			t.tris[i][1], t.tris[i][2] = t.tris[i][2], t.tris[i][1]
		}
	}
	proof, err := crossbore.ProveToolBand(ctx, s.key.R, s.key.r, d, s.key.pose, func(j int) (r3.Vec, r3.Vec) {
		return t.verts[4*j], t.verts[4*j+3]
	})
	if err != nil {
		return nil, err
	}
	t.proof = proof
	t.beta = make([]float64, len(t.verts))
	for j := range n {
		t.beta[4*j], t.beta[4*j+3] = proof.Chord, proof.Chord
	}
	var band [][3]int
	for i, f := range t.tris {
		if t.src[i] == toolBand {
			band = append(band, f)
		}
	}
	t.bandArea, t.bandSlop = heldArea(t.verts, band)
	return t, nil
}

// exactLocal returns the exact local coordinates of world point p.
func exactLocal(pose crossbore.Pose, p r3.Vec) [3]*big.Rat {
	w := [3]float64{p.X, p.Y, p.Z}
	sh := [3]float64{pose.Shift.X, pose.Shift.Y, pose.Shift.Z}
	var l [3]*big.Rat
	for i := range 3 {
		a := pose.Axis[i]
		l[i] = new(big.Rat).Sub(proofarith.FloatRat(w[a]), proofarith.FloatRat(sh[a]))
		if pose.Sign[i] < 0 {
			l[i].Neg(l[i])
		}
	}
	return l
}

// checkOutside proves, exactly on the held tool vertices, that the tool's
// three outside sides miss the solid by a margin. The sides O''→A and A→K of
// each quad lie in one half-space n·p ≥ (R + μ)|n| with n horizontal, so
// they clear the shaft's cylinder grown by μ. The side K→B'' has every vertex
// inside the bore shrunk by μ', a convex set, so the whole side does. μ and μ'
// are the walls' held bounds plus the band's chord bound plus slack, so the
// held walls and the exact band's ends clear the sides too. A failure refuses.
func checkOutside(t *saddleTool, s saddle) error {
	const n = crossbore.ToolStations
	slack := 64 * ulp * s.scale
	muBig := proofbound.AbsSumUpper(s.betaBig, t.proof.Chord, slack)
	muBore := proofbound.AbsSumUpper(s.betaBore, t.proof.Chord, slack)
	reach := proofarith.FloatRat(proofbound.AbsSumUpper(s.key.R, muBig))
	reach2 := new(big.Rat).Mul(reach, reach)
	inner := s.key.r - muBore
	if !(inner > 0) {
		return fmt.Errorf("%w: the bore's held bound leaves the saddle tool no room", decaderr.ErrUnsupported)
	}
	// Round the shrunk radius down: a vertex inside the smaller disc is inside.
	innerR := proofarith.FloatRat(math.Nextafter(inner, 0))
	inner2 := new(big.Rat).Mul(innerR, innerR)
	locals := make([][3]*big.Rat, len(t.verts))
	for i, v := range t.verts {
		locals[i] = exactLocal(s.key.pose, v)
	}
	fail := fmt.Errorf("%w: the saddle chamfer's tool does not clear the solid outside its band", decaderr.ErrUnsupported)
	for j := range n {
		next := (j + 1) % n
		for _, side := range [][2]int{{0, 1}, {1, 2}} {
			quad := [4]int{4*j + side[0], 4*j + side[1], 4*next + side[0], 4*next + side[1]}
			nx, ny := 0.0, 0.0
			for _, v := range quad {
				p := local(s.key.pose, t.verts[v])
				nx, ny = nx+p.X, ny+p.Y
			}
			bx, by := proofarith.FloatRat(nx), proofarith.FloatRat(ny)
			norm2 := new(big.Rat).Add(new(big.Rat).Mul(bx, bx), new(big.Rat).Mul(by, by))
			want := new(big.Rat).Mul(reach2, norm2)
			for _, v := range quad {
				l := locals[v]
				dot := new(big.Rat).Add(new(big.Rat).Mul(bx, l[0]), new(big.Rat).Mul(by, l[1]))
				if dot.Sign() <= 0 || new(big.Rat).Mul(dot, dot).Cmp(want) < 0 {
					return fail
				}
			}
		}
		for _, v := range []int{4*j + 2, 4*j + 3} {
			l := locals[v]
			if new(big.Rat).Add(new(big.Rat).Mul(l[1], l[1]), new(big.Rat).Mul(l[2], l[2])).Cmp(inner2) >= 0 {
				return fail
			}
		}
	}
	return nil
}

// checkSaddleClear refuses when anything but the two walls near this mouth
// could reach the tool: a facet of another face, or a segment of another
// boundary loop of either wall, whose box grown by its held bounds plus slack
// meets the tool's box. Inside that box the solid is then the region inside
// the shaft and outside the bore, so the tool's band meets the solid only in
// the chamfer's wedge.
func checkSaddleClear(fp facetedrecord.Payload, faceIndex map[*topo.Face]int, s saddle, t *saddleTool) error {
	box := t.box()
	slack := 64 * ulp * s.scale
	grown := func(b facetBox, by float64) facetBox {
		g := r3.Vec{X: by, Y: by, Z: by}
		return facetBox{lo: b.lo.Sub(g), hi: b.hi.Add(g)}
	}
	ib, iBore := faceIndex[s.big], faceIndex[s.bore]
	for i, f := range fp.Tris {
		if fp.FaceOf[i] == ib || fp.FaceOf[i] == iBore {
			continue
		}
		by := proofbound.AbsSumUpper(max(fp.VertexBound[f[0]], fp.VertexBound[f[1]], fp.VertexBound[f[2]]), slack)
		if grown(boxOf(fp.Verts, f), by).meets(box) {
			return fmt.Errorf("%w: another face of the body comes within the saddle chamfer's reach",
				decaderr.ErrUnsupported)
		}
	}
	inChain := map[int]struct{}{}
	for _, v := range s.chain {
		inChain[v] = struct{}{}
	}
	for _, fi := range []int{ib, iBore} {
		loops, err := boundaryLoops(fp.Tris, facetsOfFace(fp.FaceOf, fi))
		if err != nil {
			return err
		}
		for _, loop := range loops {
			if _, mouth := inChain[loop[0]]; mouth {
				continue
			}
			for k, v := range loop {
				w := loop[(k+1)%len(loop)]
				seg := boxOf(fp.Verts, [3]int{v, w, w})
				by := proofbound.AbsSumUpper(max(fp.VertexBound[v], fp.VertexBound[w]), slack)
				if grown(seg, by).meets(box) {
					return fmt.Errorf("%w: another boundary of the saddle's walls comes within the chamfer's reach",
						decaderr.ErrUnsupported)
				}
			}
		}
	}
	return nil
}

// toolBox is the tool's bounding box.
func (t *saddleTool) box() facetBox {
	box := boxOf(t.verts, t.tris[0])
	for _, f := range t.tris {
		box = box.join(boxOf(t.verts, f))
	}
	return box
}

// slacks returns the ring's volume and area terms. Its true solid T* is the
// held ring with its band replaced by the exact extended band, which differs
// from the held ring only between the two bands, within the chord bound: the
// volume term charges that bound over both bands' areas (Claim K), and the
// area term the difference of the two areas plus a seal of the chord bound
// along both band ends, where the exact band's edge meets the held sides.
func (t *saddleTool) slacks() (float64, float64) {
	const n = crossbore.ToolStations
	ends := 0.0
	for j := range n {
		next := (j + 1) % n
		ends = proofbound.AbsSumUpper(ends, t.verts[4*j].Sub(t.verts[4*next]).Len(),
			t.verts[4*j+3].Sub(t.verts[4*next+3]).Len())
	}
	// Each chord is a nearest-rounded length: charge its rounding too.
	ends = proofbound.ProductUpper(ends, 1+0x1p-48)
	seal := proofbound.ProductUpper(proofbound.ProductUpper(2, ends), t.proof.Chord)
	heldHi := proofbound.AbsSumUpper(t.bandArea, t.bandSlop)
	heldLo := t.bandArea - t.bandSlop
	gap := max(heldHi-t.proof.AreaLo, t.proof.AreaHi-heldLo)
	gap = proofbound.ProductUpper(gap, 1+0x1p-50)
	vol := proofbound.ProductUpper(t.proof.Chord, proofbound.AbsSumUpper(heldHi, t.proof.AreaHi))
	return vol, proofbound.AbsSumUpper(gap, seal)
}

// toolPayload assembles one tool of all the rings: ring i's band is group i
// and takes the role chamfer(i); every outside side shares the last group.
func toolPayload(tools []*saddleTool, producer provenance.Producer) (facetedrecord.Payload, error) {
	var pp facetedrecord.Payload
	side := len(tools)
	for i, t := range tools {
		base := len(pp.Verts)
		pp.Verts = append(pp.Verts, t.verts...)
		pp.VertexBound = append(pp.VertexBound, t.beta...)
		for k, f := range t.tris {
			pp.Tris = append(pp.Tris, [3]int{f[0] + base, f[1] + base, f[2] + base})
			src := side
			if t.src[k] == toolBand {
				src = i
			}
			pp.Src = append(pp.Src, src)
		}
		pp.Groups = append(pp.Groups, facetedrecord.Group{
			Origins: []provenance.Ref{{Producer: producer, Role: fmt.Sprintf("chamfer(%d)", i)}},
			Source:  facetedrecord.BlendSource(producer, i),
		})
		vol, area := t.slacks()
		pp.VolSymDiff = proofbound.AbsSumUpper(pp.VolSymDiff, vol)
		pp.AreaSlack = proofbound.AbsSumUpper(pp.AreaSlack, area)
	}
	pp.Groups = append(pp.Groups, facetedrecord.Group{
		Origins: []provenance.Ref{{Producer: producer, Role: "saddleChamferTool"}}})
	pp.MeshBound = facetproof.FacetBoundMax(pp.Tris, pp.VertexBound)
	pp.DPair = facetedrecord.SourceDiameterUpper(pp.Verts, pp.MeshBound)
	pp.Xform = r3.Identity()
	if proofbound.IsNonFinite(pp.VolSymDiff) || proofbound.IsNonFinite(pp.AreaSlack) ||
		proofbound.IsNonFinite(pp.DPair) {
		return facetedrecord.Payload{}, fmt.Errorf("%w: the saddle chamfer's tool has no finite bound",
			decaderr.ErrUnsupported)
	}
	return pp, nil
}

// cutSaddles chamfers the saddles: it builds and proves one ring per saddle,
// refuses unless each ring clears everything but its wedge and the rings'
// boxes are apart, and cuts the body by all of them at once. The tool takes
// env's producer identity and the cut the next.
func cutSaddles(ctx context.Context, env topo.Env, mesher booleaneval.OperandMesher,
	body *topo.Body, fp facetedrecord.Payload, saddles []saddle, d float64,
	faceIndex map[*topo.Face]int) (*topo.Body, error) {
	tools := make([]*saddleTool, len(saddles))
	allow := chaintol.Of(ctx)
	for i, s := range saddles {
		t, err := buildSaddleTool(ctx, s, d)
		if err != nil {
			return nil, err
		}
		if err := checkOutside(t, s); err != nil {
			return nil, err
		}
		if err := checkSaddleClear(fp, faceIndex, s, t); err != nil {
			return nil, err
		}
		// The ring must be one embedded closed surface for its true solid to
		// be the ring the proof describes: the exact crossing audit refuses any
		// two facets that meet away from their shared vertices and edges.
		if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewPollWorkBudget(ctx), t.verts, t.tris,
			loftmesh.MaxLoftAuditTriangles, crossbore.AuditScans); err != nil {
			return nil, fmt.Errorf("%w: the saddle chamfer's tool is not one embedded surface: %w",
				decaderr.ErrUnsupported, err)
		}
		for _, earlier := range tools[:i] {
			if earlier.box().meets(t.box()) {
				return nil, fmt.Errorf("%w: two selected saddles' chamfers come within each other's reach",
					decaderr.ErrUnsupported)
			}
		}
		tools[i] = t
		allow = max(allow, s.betaBig, s.betaBore, t.proof.Chord)
	}
	pp, err := toolPayload(tools, env.Ref)
	if err != nil {
		return nil, err
	}
	tool, err := facetedrecord.Build(ctx, env, pp)
	if err != nil {
		return nil, fmt.Errorf("%w: the saddle chamfer's tool fails its held audit: %w", decaderr.ErrUnsupported, err)
	}
	// The cut meets only the two walls of each saddle and the bands, so its
	// contacts touch facets whose bounds are the walls' held bounds and the
	// bands' chord bounds. The chain-depth gate compares those with the pair's
	// chord tolerance and refuses past it; that comparison admits nothing, and
	// every rim the cut makes composes from those proven bounds whatever the
	// allowance (agent-docs/faceted-vertex-bounds-design.md §5). The other
	// retained blends publish the walls' bounds on their cut vertices the same
	// way, so the cut allows them here.
	ev, err := booleaneval.Evaluate(chaintol.With(ctx, allow), meshbool.OpCut, body, tool, mesher, 1, nil, nil)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return nil, ctxErr
		}
		return nil, fmt.Errorf("%w: the saddle chamfer's cut refuses: %w", decaderr.ErrUnsupported, err)
	}
	if ev.Shift != nil {
		// A shifted retry moves the tool off the band the proof describes.
		return nil, fmt.Errorf("%w: the saddle chamfer's cut needed a shifted retry", decaderr.ErrUnsupported)
	}
	volume := measurement.Measurement{Value: units.CubicMillimeters(ev.Volume),
		Exactness: measurement.ExactnessOf(ev.VolumeBound), Bound: units.CubicMillimeters(ev.VolumeBound)}
	out, err := facetedrecord.BuildWithProof(ctx, topo.Env{Doc: env.Doc, Ref: env.Ref + 1, Alloc: env.Alloc},
		ev.Payload, ev.Audit, volume, ev.VolumeRat)
	if err != nil {
		return nil, fmt.Errorf("%w: the saddle chamfer's result fails its held audit: %w", decaderr.ErrUnsupported, err)
	}
	var warnings opdiagnostic.CompositeSummaries
	warnings.Add(ev.Diagnostics(meshbool.OpCut), 0)
	out.OperationWarnings = warnings.Records()
	return out, nil
}
