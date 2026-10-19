// Package stackedshell admits and records the exact stacked shells that run
// before route S's through-cut reading (docs/modify-general-design.md
// §3.1a-§3.1e): the nested rectangular boss, the circular boss, the
// rectangular blind pocket and the coaxial counterbore. Each builder reads only
// the two-slab stacked record and its face view, refuses any offset it cannot
// represent exactly, and builds its result through breprecord.
package stackedshell

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Call is what Shell hands a stacked builder, read from the live receiver as
// plain data.
type Call struct {
	// Inward reports an inward shell.
	Inward bool
	// Removed is the number of faces the shell removes.
	Removed int
	// CapEnd and CapStart report that the one removed face is the receiver's
	// own cap-end or cap-start face. Both are false unless Removed is 1.
	CapEnd, CapStart bool
	// T is the thickness in millimetres and TDelta its conversion bound.
	T, TDelta float64
}

// builder is one stacked shell. Its second result reports whether the stack
// is that builder's shape; an unmatched stack is left to the next route.
type builder func(context.Context, topo.Env, stackedrecord.Payload, breprecord.Payload, Call) (*topo.Body, bool, error)

// Build tries the open stacked shells in turn: the nested rectangular boss,
// the circular boss, the rectangular blind pocket and the coaxial
// counterbore. env is the result's builder environment. The second result
// reports whether one of them matched; an unmatched stack falls through to
// route S's through-cut reading.
func Build(ctx context.Context, env topo.Env, sp stackedrecord.Payload, view breprecord.Payload,
	call Call) (*topo.Body, bool, error) {
	for _, build := range []builder{rectBoss, roundBoss, blindPocket, counterbore} {
		body, matched, err := build(ctx, env, sp, view, call)
		if matched || err != nil {
			return body, matched, err
		}
	}
	return nil, false, nil
}

// Closed is the WithNoOpenings stacked route: the nested rectangular boss
// shell that keeps every face, over sp's face view, and then over the
// reversed stack when the lower slab is the narrower one
// (prismshell.ReverseClosedBossStack). A stack with no face view is unmatched,
// but a cancelled context is returned as the error.
func Closed(ctx context.Context, env topo.Env, sp stackedrecord.Payload, call Call) (*topo.Body, bool, error) {
	view, _, err := breprecord.OfStackedWithRoles(ctx, sp)
	if ctxErr := ctx.Err(); ctxErr != nil {
		return nil, false, ctxErr
	}
	if err != nil {
		return nil, false, nil //nolint:nilerr // a stack with no face view is unmatched; the caller refuses it
	}
	body, matched, err := rectBoss(ctx, env, sp, view, call)
	if matched || err != nil {
		return body, matched, err
	}
	reversed, eligible, err := prismshell.ReverseClosedBossStack(ctx, sp)
	if err != nil || !eligible {
		return nil, false, err
	}
	reversedView, _, err := breprecord.OfStackedWithRoles(ctx, reversed)
	if err != nil {
		return nil, false, err
	}
	return rectBoss(ctx, env, reversed, reversedView, call)
}

// twoSlabs reports a stack of exactly two single-region slabs under one
// interface, the only stack any builder here reads.
func twoSlabs(sp stackedrecord.Payload) bool {
	return len(sp.Slabs) == 2 && len(sp.Interfaces) == 1 &&
		len(sp.Slabs[0].Regions) == 1 && len(sp.Slabs[1].Regions) == 1
}

// exactSlabs reports zero section, level and thickness displacement, with
// the lower slab's top at the upper slab's bottom and both slabs non-empty.
func exactSlabs(sp stackedrecord.Payload, tDelta float64) bool {
	a, b := sp.Slabs[0], sp.Slabs[1]
	return sp.SectionDelta == 0 && a.Z0Delta == 0 && a.Z1Delta == 0 && b.Z0Delta == 0 && b.Z1Delta == 0 &&
		a.Z1 == b.Z0 && a.Z0 < a.Z1 && b.Z0 < b.Z1 && tDelta == 0
}

// soleCap returns the index of the one planar face of faces at level z with
// the given orientation. A non-empty second result is the refusal reason:
// no such face, or more than one, each named as what.
func soleCap(faces []breprecord.Face, z float64, outward bool, what string) (int, string) {
	idx := -1
	for i, face := range faces {
		if face.Planar() && face.Outward == outward && face.Z0 == z {
			if idx >= 0 {
				return -1, "has more than one " + what
			}
			idx = i
		}
	}
	if idx < 0 {
		return -1, "has no " + what
	}
	return idx, ""
}

// annulus is outer with hole's outer loop, reversed, as its one hole.
func annulus(ctx context.Context, outer, hole momentinput.Profile) (momentinput.Profile, error) {
	reversed, err := offset2d.ReverseLoopRecordContext(ctx, hole.Outer)
	if err != nil {
		return momentinput.Profile{}, err
	}
	outer.Holes = []sectionrecord.LoopRecord{reversed}
	return outer, nil
}

// resizedCircle is the recorded whole circle of p at radius mm. The second
// result is false when p's one segment is not a circle.
func resizedCircle(p momentinput.Profile, radius float64) (momentinput.Profile, bool) {
	circle, ok := p.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok {
		return momentinput.Profile{}, false
	}
	circle.Radius = units.Millimeters(radius)
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}, true
}

// record collects a result's faces in the stack's frame: the view's faces
// less the removed cap, then the new planes and walls in append order.
type record struct {
	frame r3.Frame
	p     breprecord.Payload
}

// newRecord copies view's faces except the one at index removed; a negative
// removed keeps them all. extra sizes the room for the faces added after.
func newRecord(sp stackedrecord.Payload, view breprecord.Payload, removed, extra int) *record {
	r := &record{frame: sp.Frame,
		p: breprecord.Payload{Xform: view.Xform, Faces: make([]breprecord.Face, 0, len(view.Faces)+extra)}}
	if removed < 0 {
		r.p.Faces = append(r.p.Faces, view.Faces...)
		return r
	}
	r.p.Faces = append(r.p.Faces, view.Faces[:removed]...)
	r.p.Faces = append(r.p.Faces, view.Faces[removed+1:]...)
	return r
}

// plane appends the planar face p at level z and returns its index.
func (r *record) plane(p momentinput.Profile, z float64, outward bool) int {
	r.p.Faces = append(r.p.Faces, breprecord.Face{Frame: r.frame, Region: &p, Z0: z, Z1: z, Outward: outward})
	return len(r.p.Faces) - 1
}

// walls appends one wall per segment of p's outer loop between levels z0
// and z1, given in either order, each segment reversed when reversed is set.
func (r *record) walls(p momentinput.Profile, z0, z1 float64, reversed bool) {
	if z0 > z1 {
		z0, z1 = z1, z0
	}
	for _, seg := range p.Outer.Segments {
		if reversed {
			seg = brepgeom.ReverseSegment(seg)
		}
		r.p.Faces = append(r.p.Faces, breprecord.Face{Frame: r.frame, Wall: seg, Z0: z0, Z1: z1})
	}
}

// build assigns the record's roles and builds it as a matched result.
func (r *record) build(ctx context.Context, env topo.Env) (*topo.Body, bool, error) {
	r.p.AssignRoles()
	body, err := breprecord.Build(ctx, env, r.p)
	return body, true, err
}
