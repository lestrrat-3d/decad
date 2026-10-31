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
// of the crossbore.ToolStations stations θ_j holds toolStride vertices:
//
//   - O⁺, the outer foot extended past the shaft wall along the band;
//   - A, O⁺ pushed out radially from the shaft's axis to R + r;
//   - K, the point (X + r, r·cos θ/2, r·sin θ/2) in the bore's extension
//     outside the shaft, with X = √(R² − r²cos²θ);
//   - B⁺, the bore foot extended into the bore along the band;
//   - the band's inner rows 1 … crossbore.ToolRows − 1 from B⁺ towards O⁺.
//
// The tool is the ring of quads joining consecutive stations over the sides
// O⁺→A→K→B⁺ and the band's rows from B⁺ back to O⁺. The rows are the chamfer
// band; the other three sides lie outside the true solid by a proven margin.

const (
	toolBand = iota
	toolSide
)

// toolStride is the number of tool vertices per station.
const toolStride = 3 + crossbore.ToolRows

// rowVertex is the index of station j's band vertex on row i: row 0 is B⁺
// and row crossbore.ToolRows is O⁺.
func rowVertex(j, i int) int {
	switch i {
	case 0:
		return toolStride*j + 3
	case crossbore.ToolRows:
		return toolStride * j
	}
	return toolStride*j + 3 + i
}

// saddleTool is the tool mesh in world coordinates, with each vertex's bound.
type saddleTool struct {
	verts []r3.Vec
	beta  []float64
	tris  [][3]int
	src   []int
	// row is the band row of each facet, −1 for an outside side.
	row   []int
	proof crossbore.ToolProof
	// rowArea[i] is the held float area of row i's facets, within rowSlop[i]
	// of their exact area.
	rowArea, rowSlop [crossbore.ToolRows]float64
}

// stationPoints returns the toolStride local vertices of station j.
func stationPoints(R, r, d float64, j int) [toolStride]r3.Vec {
	o, b := crossbore.BandEnds(R, r, d, j)
	theta := 2 * math.Pi * float64(j) / crossbore.ToolStations
	s, c := math.Sincos(theta)
	x := math.Sqrt(R*R - r*r*c*c)
	k := r3.Vec{X: x + r, Y: r * c / 2, Z: r * s / 2}
	scale := (R + r) / math.Hypot(o.X, o.Y)
	a := r3.Vec{X: o.X * scale, Y: o.Y * scale, Z: o.Z}
	p := [toolStride]r3.Vec{o, a, k, b}
	for i := 1; i < crossbore.ToolRows; i++ {
		p[3+i] = crossbore.BandRow(o, b, i)
	}
	return p
}

// placeSaddleTool places the stations and joins them; proveSaddleTool
// proves the band.
func placeSaddleTool(s saddle, d float64) (*saddleTool, error) {
	const n = crossbore.ToolStations
	t := &saddleTool{verts: make([]r3.Vec, 0, toolStride*n)}
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
		for side := range 3 {
			a, b := toolStride*j+side, toolStride*j+side+1
			c, e := toolStride*next+side+1, toolStride*next+side
			t.tris = append(t.tris, [3]int{a, b, c}, [3]int{a, c, e})
			t.src = append(t.src, toolSide, toolSide)
			t.row = append(t.row, -1, -1)
		}
		for i := range crossbore.ToolRows {
			a, b := rowVertex(j, i), rowVertex(j, i+1)
			c, e := rowVertex(next, i+1), rowVertex(next, i)
			t.tris = append(t.tris, [3]int{a, b, c}, [3]int{a, c, e})
			t.src = append(t.src, toolBand, toolBand)
			t.row = append(t.row, i, i)
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
	return t, nil
}

// proveSaddleTool proves the band of t, placed by placeSaddleTool, and sets
// each band vertex's bound and each row's held area.
func proveSaddleTool(ctx context.Context, t *saddleTool, s saddle, d float64) error {
	const n = crossbore.ToolStations
	proof, err := crossbore.ProveToolBand(ctx, s.key.R, s.key.r, d, s.key.pose, func(j, i int) r3.Vec {
		return t.verts[rowVertex(j, i)]
	})
	if err != nil {
		return err
	}
	t.proof = proof
	t.beta = make([]float64, len(t.verts))
	for j := range n {
		for i := range crossbore.ToolRows + 1 {
			t.beta[rowVertex(j, i)] = proof.Chord
		}
	}
	var rows [crossbore.ToolRows][][3]int
	for k, f := range t.tris {
		if t.row[k] >= 0 {
			rows[t.row[k]] = append(rows[t.row[k]], f)
		}
	}
	for i := range rows {
		t.rowArea[i], t.rowSlop[i] = heldArea(t.verts, rows[i])
	}
	return nil
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
// three outside sides miss the solid by a margin. The sides O⁺→A and A→K of
// each quad lie in one half-space n·p ≥ (R + μ)|n| with n horizontal, so
// they clear the shaft's cylinder grown by μ. The side K→B⁺ has every vertex
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
	// The check reads O⁺, A, K and B⁺ of each station, never the band's
	// inner rows.
	locals := make([][3]*big.Rat, len(t.verts))
	for j := range n {
		for v := toolStride * j; v < toolStride*j+4; v++ {
			locals[v] = exactLocal(s.key.pose, t.verts[v])
		}
	}
	fail := fmt.Errorf("%w: the saddle chamfer's tool does not clear the solid outside its band", decaderr.ErrUnsupported)
	for j := range n {
		next := (j + 1) % n
		for _, side := range [][2]int{{0, 1}, {1, 2}} {
			quad := [4]int{toolStride*j + side[0], toolStride*j + side[1],
				toolStride*next + side[0], toolStride*next + side[1]}
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
		for _, v := range []int{toolStride*j + 2, toolStride*j + 3} {
			l := locals[v]
			if new(big.Rat).Add(new(big.Rat).Mul(l[1], l[1]), new(big.Rat).Mul(l[2], l[2])).Cmp(inner2) >= 0 {
				return fail
			}
		}
	}
	return nil
}

// checkSaddleClear refuses when anything but the two walls near this mouth
// could reach the tool (checkMouthClear with the tool's box). Inside that box
// the solid is then the region inside the shaft and outside the bore, so the
// tool's band meets the solid only in the chamfer's wedge.
func checkSaddleClear(fp facetedrecord.Payload, faceIndex map[*topo.Face]int, s saddle, t *saddleTool) error {
	return checkMouthClear(fp, faceIndex, s, t.box(), "chamfer")
}

// checkMouthClear refuses when a facet of a face other than the saddle's two
// walls, or a segment of a boundary loop of either wall other than this
// mouth's seam, has a box grown by its held bounds plus slack that meets box.
// blend names the saddle blend in the refusal.
func checkMouthClear(fp facetedrecord.Payload, faceIndex map[*topo.Face]int, s saddle, box facetBox,
	blend string) error {
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
			return fmt.Errorf("%w: another face of the body comes within the saddle %s's reach",
				decaderr.ErrUnsupported, blend)
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
					return fmt.Errorf("%w: another boundary of the saddle's walls comes within the %s's reach",
						decaderr.ErrUnsupported, blend)
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
// area term, row by row, the gap between the row's held area and its exact
// interval, plus a seal of the chord bound along both band ends, where the
// exact band's edge meets the held sides.
func (t *saddleTool) slacks() (float64, float64) {
	const n = crossbore.ToolStations
	ends := 0.0
	for j := range n {
		next := (j + 1) % n
		ends = proofbound.AbsSumUpper(ends, t.verts[toolStride*j].Sub(t.verts[toolStride*next]).Len(),
			t.verts[toolStride*j+3].Sub(t.verts[toolStride*next+3]).Len())
	}
	// Each chord is a nearest-rounded length: charge its rounding too.
	ends = proofbound.ProductUpper(ends, 1+0x1p-48)
	seal := proofbound.ProductUpper(proofbound.ProductUpper(2, ends), t.proof.Chord)
	gap, heldHi := 0.0, 0.0
	for i := range crossbore.ToolRows {
		hi := proofbound.AbsSumUpper(t.rowArea[i], t.rowSlop[i])
		lo := math.Nextafter(t.rowArea[i]-t.rowSlop[i], math.Inf(-1))
		heldHi = proofbound.AbsSumUpper(heldHi, hi)
		gap = proofbound.AbsSumUpper(gap, max(hi-t.proof.RowLo[i], t.proof.RowHi[i]-lo, 0))
	}
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
		t, err := placeSaddleTool(s, d)
		if err != nil {
			return nil, err
		}
		// The clearance check reads only the tool's box, so it runs before
		// the band proof, the costliest step.
		if err := checkSaddleClear(fp, faceIndex, s, t); err != nil {
			return nil, err
		}
		if err := proveSaddleTool(ctx, t, s, d); err != nil {
			return nil, err
		}
		if err := checkOutside(t, s); err != nil {
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
