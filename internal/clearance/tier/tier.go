// Package tier evaluates vertex-face and vertex-edge distance cells.
package tier

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/clearance/spine"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Kernel holds one tier calculation's tolerance and any spine error.
type Kernel struct {
	tol, slack float64
	ctx        context.Context //nolint:containedctx // The kernel is per-call state.
	err        error
	refused    bool
}

// New creates one clearance tier kernel for the caller's pair.
func New(ctx context.Context, tol, slack float64) *Kernel {
	return &Kernel{ctx: ctx, tol: tol, slack: slack}
}
func (k *Kernel) Err() error               { return k.err }
func (k *Kernel) Refused() bool            { return k.refused }
func (k *Kernel) oracle() clearance.Oracle { return clearance.Oracle{Tol: k.tol} }

func (k *Kernel) pointCircleCrits(p, c, axis, refU, refV r3.Vec, rad float64, win clearance.AngWindow) ([]clearance.SpineCrit, bool) {
	e := &spine.Engine{Context: k.ctx, Tolerance: k.tol, Slack: k.slack}
	out, ok := e.PointCircleCrits(p, c, axis, refU, refV, rad, win)
	if e.Err != nil {
		k.err = e.Err
	}
	k.refused = k.refused || e.Refused
	return out, ok
}

// EnumerateVertex visits one vertex against the other body's faces, then
// edges. Each admitted cell has its own kernel, as in the pair's other tiers.
// The budget, sink, and each face and edge must be non-nil.
func (k *Kernel) EnumerateVertex(budget *proofbound.WorkBudget, v r3.Vec,
	faces []*clearance.CFace, edges []*clearance.CEdge, sink *clearance.CellSink) error {
	at := [2]r3.Vec{v, v}
	for _, f := range faces {
		if err := budget.Step(); err != nil {
			return err
		}
		if sink.Pruned(clearance.ClrBoxDist(at, f.Box)) {
			continue
		}
		cell := New(k.ctx, k.tol, k.slack)
		err := cell.VertexFace(budget, v, f, sink)
		k.captureCell(cell)
		if err != nil {
			return err
		}
	}
	for _, e := range edges {
		if err := budget.Step(); err != nil {
			return err
		}
		if sink.Pruned(clearance.ClrBoxDist(at, e.Box)) {
			continue
		}
		cell := New(k.ctx, k.tol, k.slack)
		cell.VertexEdge(v, e, sink)
		k.captureCell(cell)
	}
	return nil
}

func (k *Kernel) captureCell(cell *Kernel) {
	if err := cell.Err(); err != nil {
		k.err = err
	}
	k.refused = k.refused || cell.Refused()
}

// VertexFace contributes the point-to-face candidates to sink.
func (k *Kernel) VertexFace(budget *proofbound.WorkBudget, v r3.Vec, f *clearance.CFace, sink *clearance.CellSink) error {
	switch f.Kind {
	case clearance.CkPlane:
		h := v.Sub(f.O).Dot(f.N)
		foot := v.Sub(f.N.Scale(h))
		x, y := f.PlaneCoords(foot)
		admit, err := clearance.RegionClassifyBudget(budget, f.Region, x, y, k.tol)
		if err != nil {
			return err
		}
		if admit == -1 {
			// The sink discards a foot the trim excludes before it reads the
			// distance, so the exact reading below is skipped, not changed.
			return nil
		}
		d := clearance.Height(v, f.O, f.N).Abs().Widen(clearance.DirCharge([]r3.Vec{f.N}, []r3.Vec{v, f.O}))
		sink.CandidateDist(k.tol, admit, d, foot, v)
	case clearance.CkCone:
		rel := v.Sub(f.Anchor)
		z := rel.Dot(f.Axis)
		perp := rel.Sub(f.Axis.Scale(z))
		rho := perp.Len()
		// The meridian reading comes off the cone's slope; its float form only
		// places the foot, and the published distance is ConeDist's enclosure.
		_, t := f.ConeMeridian(z, rho)
		d := f.ConeDist(v).Widen(clearance.DirCharge([]r3.Vec{f.Axis}, []r3.Vec{v, f.Anchor}))
		sinA, cosA := f.ConeSinCos()
		var radial r3.Vec
		switch k.oracle().OnAxis(v, f.Anchor, f.Axis) {
		case clearance.DegYes:
			// Provenly on the axis: every azimuth carries the same distance, so
			// the sweep window's midpoint represents the family.
			mid := 0.0
			if !f.Sweep.Full {
				mid = (f.Sweep.Lo + f.Sweep.Hi) / 2
			}
			radial = f.RefU.Scale(math.Cos(mid)).Add(f.RefV.Scale(math.Sin(mid)))
		case clearance.DegNo:
			radial, _ = perp.Normalize()
		default:
			// The distance is azimuth-free, the admission foot is not. The
			// carrier distance is a proven lower bound; it stands as one.
			sink.LoOnly(d.Lo)
			return nil
		}
		if t <= k.tol {
			return nil // the apex holds the nearest point; the vertex tiers pair with it
		}
		pf := f.Anchor.Add(f.Axis.Scale(t * cosA)).Add(radial.Scale(t * sinA))
		sink.CandidateDist(k.tol, f.AdmitPoint(pf, k.tol), d, pf, v)
	default:
		d, foot := clearance.SpineDistOf(f, v)
		if d <= k.tol {
			sink.Unsure = true
			return nil
		}
		spineDist := clearance.SpineDist(f, v)
		dir := v.Sub(foot).Scale(1 / d)
		for _, sf := range []float64{1, -1} {
			pf := foot.Add(dir.Scale(sf * f.Radius))
			sink.CandidateDist(k.tol, f.AdmitPoint(pf, k.tol), spineDist.Sub(sf*f.Radius).Abs(), pf, v)
		}
	}
	return nil
}

// VertexEdge contributes the point-to-edge candidates to sink.
func (k *Kernel) VertexEdge(v r3.Vec, e *clearance.CEdge, sink *clearance.CellSink) {
	if e.Line {
		dir := e.B.Sub(e.A)
		u, ok := dir.Normalize()
		if !ok {
			return
		}
		foot := clearance.LinePoint(e.A, u, v)
		seg, okSeg := clearance.SegDir(e.A, e.B)
		if !okSeg {
			return
		}
		admit := clearance.LineParamAdmit(e, foot, k.tol)
		if admit == -1 {
			// As in VertexFace: the sink would discard the candidate unread.
			return
		}
		sink.CandidateDist(k.tol, admit, clearance.PointLineDist(v, e.A, seg), foot, v)
		return
	}
	crits, ok := k.pointCircleCrits(v, e.Center, e.Axis, e.RefU, e.RefV, e.Radius, e.Ang)
	if !ok {
		// The radial direction is not resolvable, so the arc's own admission
		// cannot be decided — but the distance to the WHOLE circle bounds the
		// distance to any arc of it from below, and that is a proof.
		d := clearance.PointCircleDist(v, e.Center, e.Axis, e.Radius, 1).
			Widen(clearance.DirCharge([]r3.Vec{e.Axis}, []r3.Vec{v, e.Center}, e.Radius))
		sink.LoOnly(d.Lo)
		return
	}
	for _, c := range crits {
		th := clearance.AngleOf(e, c.Fb.Sub(e.Center))
		sink.Candidate(k.tol, clearance.CircleAngleAdmit(e, th, k.tol), c.Lo, c.Hi, c.Exact, c.Fb, v)
	}
}
