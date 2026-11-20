package decad

import (
	"context"
	"maps"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/rimextension"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func upperNativeRim(ctx context.Context, d *Document, body *Body, edge *topo.Edge, setback float64) (brepmodify.Result, bool, error) {
	var none brepmodify.Result
	if !edge.Convex || edge.Start == nil || edge.Start != edge.End || !edge.RetainedCurve || len(edge.Faces) != 2 {
		return none, false, nil
	}
	if _, ok := edge.Curve.(surfacegeom.Circle3); !ok {
		return none, false, nil
	}
	if _, suppressed := ctx.Value(operationHistorySuppressedKey{}).(*operationHistoryCapture); suppressed {
		return none, false, nil
	}
	budget := proofbound.NewStrictReplayBudget()
	h, admitted, e := rimHistorySnapshot(ctx, d, budget)
	if e != nil {
		return none, false, e
	}
	if !admitted {
		return none, false, nil
	}
	history := pairedHistorySource{h}
	moves := map[*topo.Body]r3.Vec{}
	var tools, targets []*topo.Body
	cur := &body.n
	for len(tools) <= 16 {
		if err := ctx.Err(); err != nil {
			return none, false, err
		}
		if e := budget.SpendWork(ctx, uint64(8*(len(h.operations)+1))); e != nil {
			return none, false, e
		}
		kind, inputs, ok := history.BooleanProducer(cur)
		if !ok {
			return none, false, nil
		}
		if fp, yes := cur.Payload.(facetedrecord.Payload); !yes || fp.PointCone != nil {
			return none, false, nil
		}
		if !boolchain.WarningsMoveNoGeometry(cur) {
			c, _ := cur.PrivateShift.(*booleaneval.ShiftCertificate)
			v, e := c.CommittedRimTranslation(ctx, cur, inputs[0], inputs[1], kind, budget)
			if e != nil {
				return none, false, e
			}
			moves[inputs[1]] = v
		}
		if kind == meshbool.OpCut {
			tools = append(tools, inputs[1])
			targets = append(targets, inputs[0])
			cur = inputs[0]
			continue
		}
		if kind != meshbool.OpUnion {
			return none, false, nil
		}
		for i, pad := range inputs {
			if i == 0 && moves[inputs[1]] != (r3.Vec{}) {
				continue
			}
			pp, ok := pad.Payload.(prismrecord.Payload)
			if !ok || !pad.Solid || pp.SurfaceResult || pp.Xform != r3.Identity() || pp.SectionDelta != 0 || len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 {
				continue
			}
			seg, ok := pp.Profile.Outer.Segments[0].(momentinput.CircleSeg)
			if !ok {
				continue
			}
			var finalCap, finalWall *topo.Face
			var src *topo.Edge
			var level *big.Rat
			for _, f := range topobuild.Faces(pad) {
				pl, ok := f.Surface.(surfacegeom.Plane)
				if !ok || pl.Frame.N() != (r3.Vec{Y: 1}) || len(f.Origins) != 1 || len(f.Loops) != 1 || len(f.Loops[0].CoEdges) != 1 {
					continue
				}
				token, ok := f.PrismConstruction()
				if !ok || !token.MatchesSealed(f) {
					continue
				}
				sealed, se := token.ReplayMatchesTopology(ctx, f, budget)
				if se != nil {
					return none, false, se
				}
				if !sealed {
					continue
				}
				for _, final := range edge.Faces {
					if final.HasOrigin(f.Origins[0]) && final.RetainedTag && rimHistoryFace(final, targets, pad, i) == f {
						finalCap = final
						src = f.Loops[0].CoEdges[0].Edge
						level = upperRimLevel(f.CapRecord)
					}
				}
			}
			if src == nil || level == nil {
				continue
			}
			if _, ok := src.Curve.(surfacegeom.Circle3); !ok || src.Start != src.End {
				continue
			}
			matchedWall := false
			for _, wall := range src.Faces {
				if _, ok := wall.Surface.(surfacegeom.Cylinder); !ok {
					continue
				}
				token, ok := wall.PrismConstruction()
				if !ok || !token.Matches(wall, pad, pad.Origin.Producer, topo.PrismFaceWall, 0, 0) || !token.MatchesSealed(wall) {
					continue
				}
				sealed, se := token.ReplayMatchesTopology(ctx, wall, budget)
				if se != nil {
					return none, false, se
				}
				if !sealed {
					continue
				}
				for _, final := range edge.Faces {
					if _, ok := final.Surface.(surfacegeom.Cylinder); ok && final.RetainedTag && rimHistoryFace(final, targets, pad, i) == wall {
						finalWall = final
						matchedWall = true
					}
				}
			}
			if !matchedWall {
				continue
			}
			clean := true
			for _, tool := range tools {
				if !measurement.BoxesDisjoint(rimMovedBounds(tool, moves[tool]), rimMovedBounds(pad, moves[pad])) {
					clean = false
					break
				}
			}
			if !clean {
				continue
			}
			other := inputs[1-i]
			hinge := new(big.Rat).Sub(level, new(big.Rat).SetFloat64(setback))
			hinge.Add(hinge, new(big.Rat).SetFloat64(moves[pad].Y))

			radius, ue := seg.Radius.In(units.Millimeter)
			if ue != nil || seg.Center != (sectionrecord.Point2{}) || pp.Frame.U() != (r3.Vec{X: 1}) || pp.Frame.V() != (r3.Vec{Z: 1}) {
				continue
			}
			sealed, se := pad.ReplayPrismBuildPayload(ctx, pp.Frame, pp.Xform, pp.Z0, pp.Z1, pp.Z0Delta, pp.Z1Delta, pp.SectionDelta, pp.Z0Denoted, pp.Z1Denoted, []sectionrecord.AnalyticProfileView{pp.Profile.AnalyticSealView()}, budget)
			if se != nil {
				return none, false, se
			}
			if !sealed {
				continue
			}
			support := rimMovedBounds(pad, moves[pad])
			support.Min.X = math.Nextafter(support.Min.X-radius/8, math.Inf(-1))
			support.Min.Z = math.Nextafter(support.Min.Z-radius/8, math.Inf(-1))
			support.Max.X = math.Nextafter(support.Max.X+radius/8, math.Inf(1))
			support.Max.Z = math.Nextafter(support.Max.Z+radius/8, math.Inf(1))
			p0, yes := other.Payload.(facetedrecord.Payload)
			if !yes || len(p0.Tris) > 65536 {
				return none, false, nil
			}
			if e := budget.ReserveBundle(ctx, uint64(8*len(p0.Tris)), uint64(len(p0.Groups)), uint64(64*len(p0.Groups))); e != nil {
				return none, false, e
			}
			if !upperRimStock(ctx, other, support, hinge) {
				continue
			}
			center := rimVec(pp.Frame.Origin())
			move := rimVec(moves[pad])
			for k := range 3 {
				center[k] = rimAdd(center[k], move[k])
			}
			lo, hi := pp.DenotedZ0(), pp.DenotedZ1()
			if lo == nil || hi == nil || new(big.Rat).Abs(rimSub(hi, lo)).Cmp(rimRat(setback)) <= 0 {
				continue
			}
			height := proofbound.RatFloatUp(new(big.Rat).Abs(rimSub(hi, lo)))
			result, ok, err := directUpperRim(ctx, envOf(body.n.Doc, d.nextProducerID()), &body.n, finalCap, finalWall, center, hinge, radius, setback, height, budget)
			if ok && err == nil {
				target := bodyHandle(result.Body)
				top := rimAdd(hinge, rimRat(setback))
				bottom := rimSub(top, new(big.Rat).Abs(rimSub(hi, lo)))
				cone := upperRimCone(result.Body)
				if cone == nil {
					return result, ok, err
				}
				target.upperRimProof = &upperRimProof{
					stock: body, result: target, cone: cone, pad: pad,
					wall: rimHistoryFace(finalWall, targets, pad, i), sourceWall: finalWall, shift: moves[pad],
					geometry: rimextension.OuterAdmission{
						CenterX: center[0], CenterZ: center[2], Hinge: hinge, PadTop: top,
						PadBottom: bottom, Radius: rimRat(radius), NoseSetback: rimRat(setback),
					},
				}
			}
			return result, ok, err
		}
		break
	}
	return none, false, nil
}

func rimHistoryFace(f *topo.Face, targets []*topo.Body, pad *topo.Body, side int) *topo.Face {
	for i := 0; i <= len(targets); i++ {
		owner, operand := pad, side
		if i < len(targets) {
			owner, operand = targets[i], 0
		}
		p := f.Source
		if p == nil || p.Blend || len(p.Members) != 0 || p.Producer != owner.Origin.Producer || p.Operand != operand {
			return nil
		}
		fs := topobuild.Faces(owner)
		if p.Ordinal < 0 || p.Ordinal >= len(fs) || p.Parent != fs[p.Ordinal].Source {
			return nil
		}
		f = fs[p.Ordinal]
	}
	return f
}
func upperRimLevel(r *topo.CapRecord) *big.Rat {
	if r != nil && r.ZDenoted != nil && (r.ZDenoted.Num().BitLen() > 4096 || r.ZDenoted.Denom().BitLen() > 4096) {
		return nil
	}
	if r == nil || r.Xform != r3.Identity() || r.Frame.U().Y != 0 || r.Frame.V().Y != 0 || math.Abs(r.Frame.N().Y) != 1 || r.ZDenoted == nil {
		return nil
	}
	z := new(big.Rat).Mul(r.ZDenoted, new(big.Rat).SetFloat64(r.Frame.N().Y))
	return z.Add(z, new(big.Rat).SetFloat64(r.Frame.Origin().Y))
}

// Every vertical ray that meets a bounded solid has an uppermost boundary.
// An open occupied region has a positive-area upper boundary projection.
// Vertical planes and vertical cylinders have zero projection area, so they
// cannot enclose any stock above the certified horizontal cap in the box.
func upperRimStock(ctx context.Context, stock *topo.Body, b measurement.Box, hinge *big.Rat) bool {
	fp, ok := stock.Payload.(facetedrecord.Payload)
	if !ok || len(fp.Src) != len(fp.Tris) || !stock.Solid {
		return false
	}

	err := math.Nextafter(b.Bound.Base()+fp.MeshBound, math.Inf(1))
	loX := math.Nextafter(b.Min.X-err, math.Inf(-1))
	hiX := math.Nextafter(b.Max.X+err, math.Inf(1))
	loZ := math.Nextafter(b.Min.Z-err, math.Inf(-1))
	hiZ := math.Nextafter(b.Max.Z+err, math.Inf(1))
	h, _ := hinge.Float64()
	floor := math.Nextafter(math.Nextafter(h, math.Inf(-1))-err, math.Inf(-1))
	if !(err >= 0) || math.IsInf(err, 0) {
		return false
	}
	ey := r3.Vec{Y: 1}
	vertical := func(v r3.Vec) bool { return v == ey || v == ey.Scale(-1) }
	checked := map[int]bool{}
	for i, t := range fp.Tris {
		if i%256 == 0 && ctx.Err() != nil {
			return false
		}
		a, c, d := fp.Verts[t[0]], fp.Verts[t[1]], fp.Verts[t[2]]
		if max(a.Y, c.Y, d.Y) < floor || min(a.X, c.X, d.X) > hiX || max(a.X, c.X, d.X) < loX || min(a.Z, c.Z, d.Z) > hiZ || max(a.Z, c.Z, d.Z) < loZ {
			continue
		}
		n := fp.Src[i]
		if checked[n] {
			continue
		}
		checked[n] = true
		g := fp.Groups[n]
		if !g.Retained || g.Denoted != nil || g.NormalBound != 0 || g.ExtentDelta != 0 {
			return false
		}
		switch s := g.Surface.(type) {
		case surfacegeom.Plane:
			if vertical(s.Frame.U()) || vertical(s.Frame.V()) {
				continue
			}
			lev := upperRimLevel(g.CapRecord)
			if lev == nil || lev.Cmp(hinge) >= 0 {
				return false
			}
		case surfacegeom.Cylinder:
			if !vertical(s.Axis) {
				return false
			}
		default:
			return false
		}
	}
	return true
}

func rimMovedBounds(b *topo.Body, v r3.Vec) measurement.Box {
	box := b.Bounds
	// Union of the original and shifted projection is sufficient for this exclusion.
	box.Min = r3.Vec{X: math.Nextafter(box.Min.X+min(0, v.X), math.Inf(-1)), Y: math.Nextafter(box.Min.Y+min(0, v.Y), math.Inf(-1)), Z: math.Nextafter(box.Min.Z+min(0, v.Z), math.Inf(-1))}
	box.Max = r3.Vec{X: math.Nextafter(box.Max.X+max(0, v.X), math.Inf(1)), Y: math.Nextafter(box.Max.Y+max(0, v.Y), math.Inf(1)), Z: math.Nextafter(box.Max.Z+max(0, v.Z), math.Inf(1))}
	return box
}

// rimHistorySnapshot copies only while the document lock protects both the
// storage count and the subsequent copy. The same meter funds the face walk.
func rimHistorySnapshot(ctx context.Context, d *Document, b *proofbound.ReplayBudget) (*OperationHistory, bool, error) {
	d.n.LiveMu.RLock()
	defer d.n.LiveMu.RUnlock()
	n := len(d.operations) + len(d.producers) + len(d.consumers)
	if n > 65536 {
		return nil, false, nil
	}
	for _, v := range d.consumers {
		if len(v) > 65536-n {
			return nil, false, nil
		}
		n += len(v)
	}
	if err := b.ReserveBundle(ctx, uint64(n), uint64(n), uint64(128*n)); err != nil {
		return nil, false, err
	}
	h := &OperationHistory{doc: &d.n, operations: slices.Clone(d.operations), producers: maps.Clone(d.producers), consumers: make(map[*Body][]*Operation, len(d.consumers))}
	for k, v := range d.consumers {
		h.consumers[k] = slices.Clone(v)
	}
	return h, true, nil
}

func upperRimCone(result *topo.Body) *topo.Face {
	var cone *topo.Face
	for _, f := range topobuild.Faces(result) {
		src := f.Source
		if src != nil && src.Blend && src.Producer == result.Origin.Producer && src.Ordinal == 0 {
			if cone != nil {
				return nil
			}
			cone = f
		}
	}
	return cone
}
