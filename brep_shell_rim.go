package decad

import (
	"context"
	"fmt"
	"slices"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/throughshell"
	"github.com/lestrrat-3d/r3"
)

// This file is route S's rim assembly (docs/modify-general-design.md §3.3
// steps 5 and 6, "modify-general §N" below). Each removed face R is stated as
// a planar face, and its rim is R's region less the cavity's trace in R's
// plane: R's outer loop holding every cavity face there as a hole, and one
// band per hole of R between that hole and the cavity hole it partners.
// Where the cavity's trace runs along R's own boundary — a removed wall run
// beside a removed cap, each rim reaching the other's edge — the coincident
// pieces cancel and the rest chain into the rim's loops. Every comparison is
// exact on recorded floats, and every loop the assembly cannot state refuses
// (SG7).

// throughCutRims assembles the shell's record (modify-general §3.3 steps 5
// and 6): the receiver's faces but the removed ones, verbatim; the cavity's
// faces reversed, except those lying in a removed face's plane; and, per
// removed face R, its rim: R's region less every cavity face Q in R's plane
// (throughRimRegions), and for each hole h of R the band between h and the
// hole of some Q that partners it — at a cap the hole of the eroded section
// of h's index, at a pierced wall the section of the tool through h dilated
// by t. A loop with no partner is SG7. Every rim region and band runs modify
// §5's audit (S8, S7, S9). A cap's rim carries the cavity's section
// displacement; a wall's rim, whose region states the cavity's levels and the
// receiver's cap levels as in-plane coordinates, carries the largest of that
// and every cavity level's displacement.
func throughCutRims(ctx context.Context, budget *proofbound.WorkBudget, bp brepPayload, tc throughCut, cavity brepPayload, eroded ProfileRecord, dilated []ProfileRecord, rm throughRemoval) (brepPayload, error) {
	frames := make([]r3.Frame, 0, 1+len(cavity.faces))
	frames = append(frames, bp.faces[0].frame)
	for _, f := range cavity.faces {
		frames = append(frames, f.frame)
	}
	embeds, err := brepgeom.Embeds(frames, ErrUnsupported)
	if err != nil {
		return brepPayload{}, err
	}
	cavEmbeds := embeds[1:]
	joined, _, err := brepJoinProfile(eroded)
	if err != nil {
		return brepPayload{}, err
	}
	charge := cavity.sectionDelta()
	for _, f := range cavity.faces {
		charge = max(charge, f.z0Delta, f.z1Delta)
	}

	var removed []int
	if rm.bottom {
		removed = append(removed, tc.caps.Bottom)
	}
	if rm.top {
		removed = append(removed, tc.caps.Top)
	}
	removed = append(removed, rm.walls...)
	gone := map[int]struct{}{}
	inPlane := map[int]struct{}{}
	var rims []brepFace
	for _, fi := range removed {
		if err := ctx.Err(); err != nil {
			return brepPayload{}, err
		}
		gone[fi] = struct{}{}
		rp, err := tc.rimPlane(bp, fi)
		if err != nil {
			return brepPayload{}, err
		}
		r := rp.face
		partner, err := tc.rimPartner(ctx, fi, rp, joined, dilated)
		if err != nil {
			return brepPayload{}, err
		}
		var qOuters []LoopRecord
		var bands []ProfileRecord
		partnered := map[int]struct{}{}
		// Each cavity face in R's plane: a planar face across R's axis at its
		// level, or at a wall, a straight cavity wall along k on R's carrier.
		for ci, q := range cavity.faces {
			eQ := cavEmbeds[ci]
			if !q.planar() {
				if !rp.wall || eQ.Axis[2] != tc.k {
					continue
				}
				outer, ok := throughshell.SweptTrace(q.wall, q.z0, q.z1, eQ, rp.e, rp.axis, rp.level, tc.k)
				if !ok {
					continue
				}
				inPlane[ci] = struct{}{}
				qOuters = append(qOuters, outer)
				continue
			}
			if eQ.Axis[2] != rp.axis || brepLevel(q, eQ) != rp.level {
				continue
			}
			inPlane[ci] = struct{}{}
			inR, ok := brepgeom.NewPrismMap(eQ, rp.e).Region(*q.region)
			if !ok {
				return brepPayload{}, throughRimError(r, "a cavity face in its plane does not map into its frame")
			}
			inF, ok := brepgeom.NewPrismMap(eQ, tc.caps.Embed).Region(*q.region)
			if !ok && !rp.wall {
				return brepPayload{}, throughRimError(r, "a cavity face in its plane does not map into the prism's frame")
			}
			qOuters = append(qOuters, inR.Outer)
			for qh, hole := range inR.Holes {
				var holeF LoopRecord
				if !rp.wall {
					holeF = inF.Holes[qh]
				}
				hi, err := partner(hole, holeF)
				if err != nil {
					return brepPayload{}, err
				}
				if _, dup := partnered[hi]; dup {
					return brepPayload{}, throughRimError(r, "a cavity hole partners no hole of the removed face, or two")
				}
				partnered[hi] = struct{}{}
				band, err := offset2d.ReverseLoopRecordContext(ctx, hole)
				if err != nil {
					return brepPayload{}, err
				}
				bands = append(bands, ProfileRecord{Outer: band, Holes: []LoopRecord{r.region.Holes[hi]}})
			}
		}
		if len(partnered) != len(r.region.Holes) {
			return brepPayload{}, throughRimError(r, "a hole of the removed face has no cavity hole to partner")
		}
		regions, err := throughRimRegions(ctx, budget, r, r.region.Outer, qOuters)
		if err != nil {
			return brepPayload{}, err
		}
		delta := cavity.sectionDelta()
		if rp.wall {
			delta = charge
		}
		for _, region := range append(regions, bands...) {
			if err := auditThroughRim(budget, region); err != nil {
				return brepPayload{}, err
			}
			region := region
			rims = append(rims, brepFace{frame: r.frame, region: &region, outward: r.outward, sweep: r.sweep,
				z0: r.z0, z1: r.z0, z0Delta: r.z0Delta, z1Delta: r.z0Delta, delta: delta})
		}
	}

	out := brepPayload{xform: bp.xform}
	for fi, f := range bp.faces {
		if _, removed := gone[fi]; removed {
			continue
		}
		out.faces = append(out.faces, f)
	}
	for ci, f := range cavity.faces {
		if _, in := inPlane[ci]; in {
			continue
		}
		out.faces = append(out.faces, f.reversed())
	}
	out.faces = append(out.faces, rims...)
	return out, nil
}

// throughRimPlane is one removed face stated as a planar face: face is the
// statement (frame, region, outward, level, sweep), e its frame's embed, axis
// and level its plane in reference coordinates, and wall whether it is a wall
// of A rather than a cap.
type throughRimPlane struct {
	face  brepFace
	e     brepEmbed
	axis  int
	level float64
	wall  bool
}

// rimPlane states removed face fi as a planar face. A cap and a pierced wall
// are planar faces already and are taken as recorded. A wall along k is a
// straight wall along a section axis (removedFaces admits no other): its
// statement is the rectangle it sweeps over [zlo, zhi] in the signed
// permutation frame across its normal axis whose normal is its outward
// normal, restating a wall along k as a planar face does (brepFace.sweep).
func (tc throughCut) rimPlane(bp brepPayload, fi int) (throughRimPlane, error) {
	f, e := bp.faces[fi], tc.embeds[fi]
	if f.planar() {
		return throughRimPlane{face: f, e: e, axis: e.Axis[2], level: brepLevel(f, e), wall: tc.kinds[fi] == throughPierced}, nil
	}
	wall, reason := throughshell.RestateWall(f.wall, e, bp.faces[0].frame, tc.k, tc.caps.Zlo, tc.caps.Zhi)
	if reason != "" {
		return throughRimPlane{}, throughRimError(f, reason)
	}
	z := wall.Embed.Sign[2]*wall.Level + 0
	face := brepFace{frame: wall.Frame, region: &wall.Region, outward: true,
		sweep: tc.caps.Frame.N(), z0: z, z1: z, role: f.role}
	return throughRimPlane{face: face, e: wall.Embed, axis: wall.Axis, level: wall.Level, wall: true}, nil
}

// rimPartner returns the hole partnering of removed face fi: given a cavity
// face's hole in R's frame (and, at a cap, in the prism frame F), the index
// of the hole of R it partners, or SG7. At a cap the cavity hole must equal
// the eroded section's hole of some index (S-1's rule: offsetProfile keeps
// loop order, and the joined record is what class B states), and R's hole of
// that section index partners it. At a pierced wall the cavity hole must
// equal the section of the tool through one of R's holes dilated by t, as the
// tool's own cut states it: the joined dilated section carried into R's frame
// and reversed, compared as throughshell.LoopsSame reads two loops. A wall
// along k holds no hole, so any cavity hole there is SG7.
func (tc throughCut) rimPartner(ctx context.Context, fi int, rp throughRimPlane, joined ProfileRecord, dilated []ProfileRecord) (func(inR, inF LoopRecord) (int, error), error) {
	r := rp.face
	miss := func(what string) (int, error) { return -1, throughRimError(r, what) }
	if !rp.wall {
		// The section hole index of each of R's region holes.
		holes := make([]int, len(r.region.Holes))
		for hi := range holes {
			holes[hi] = hi
			if fi == tc.caps.Bottom {
				holes[hi] = tc.caps.BottomLoop[1+hi] - 1
			}
		}
		return func(_, inF LoopRecord) (int, error) {
			si := slices.IndexFunc(joined.Holes, func(want LoopRecord) bool { return brepgeom.LoopsEqual(want, inF) })
			if si < 0 {
				return miss("a cavity face in its plane holds a loop that is neither its outer loop nor a hole of the eroded section")
			}
			hi := slices.Index(holes, si)
			if hi < 0 {
				return miss("a hole of the eroded section partners no hole of the removed face")
			}
			return hi, nil
		}, nil
	}
	type want struct {
		loop LoopRecord
		hi   int
	}
	var wants []want
	for i, tool := range tc.tools {
		var hi int
		switch fi {
		case tool.w0:
			hi = tool.loop0 - 1
		case tool.w1:
			hi = tool.loop1 - 1
		default:
			continue
		}
		section, _, err := brepJoinProfile(dilated[i])
		if err != nil {
			return nil, err
		}
		inR, ok := brepgeom.NewPrismMap(tool.frameEmb, rp.e).Loop(section.Outer)
		if !ok {
			return nil, throughRimError(r, "a dilated tool's section does not map into its frame")
		}
		loop, err := offset2d.ReverseLoopRecordContext(ctx, inR)
		if err != nil {
			return nil, err
		}
		wants = append(wants, want{loop: loop, hi: hi})
	}
	return func(inR, _ LoopRecord) (int, error) {
		for _, w := range wants {
			if throughshell.LoopsSame(w.loop, inR) {
				return w.hi, nil
			}
		}
		return miss("a cavity face in its plane holds a loop that is neither its outer loop nor the dilated section of a tool through it")
	}, nil
}

// throughRimRegions classifies the loops left by the cavity trace. The
// geometry and exact cancellation are computed in internal/brepgeom; the
// signed-area budget and SG7 refusal belong to the shell operation.
func throughRimRegions(ctx context.Context, budget *proofbound.WorkBudget, r brepFace,
	outer LoopRecord, cavity []LoopRecord) ([]ProfileRecord, error) {
	trace, err := brepgeom.TraceRim(ctx, outer, cavity)
	if err != nil {
		if reason, ok := err.(brepgeom.RimTraceError); ok {
			return nil, throughRimError(r, reason.Error())
		}
		return nil, err
	}
	if !trace.Cancelled {
		return []ProfileRecord{{Outer: trace.Loops[0], Holes: trace.Loops[1:]}}, nil
	}
	var outs, inner []LoopRecord
	for _, rec := range trace.Loops {
		area, err := loopSignedAreaBudget(budget, rec)
		if err != nil {
			return nil, shellCancelCause(err)
		}
		switch {
		case area > 0:
			outs = append(outs, rec)
		case area < 0:
			inner = append(inner, rec)
		default:
			return nil, throughRimError(r, "a loop left by the cavity's trace encloses no area")
		}
	}
	switch {
	case len(inner) == 0:
		out := make([]ProfileRecord, len(outs))
		for i, o := range outs {
			out[i] = ProfileRecord{Outer: o}
		}
		return out, nil
	case len(outs) == 1:
		return []ProfileRecord{{Outer: outs[0], Holes: inner}}, nil
	}
	return nil, throughRimError(r, "the loops left by the cavity's trace are not one outer loop with holes")
}

// throughRimError is SG7, naming the removed face.
func throughRimError(r brepFace, what string) error {
	return fmt.Errorf(`%w: the shell's rim at %s does not partner the cavity's trace: %s (modify-general SG7)`, ErrUnsupported, r.role, what)
}

// auditThroughRim is modify §5's audit on one rim region or band: S8 (the
// outer loop walks counter-clockwise and every hole clockwise, each with a
// non-zero area), then S7 (no crossing or contact) and S9 (nesting) through
// the shared audit.
func auditThroughRim(budget *proofbound.WorkBudget, region ProfileRecord) error {
	for li, loop := range append([]LoopRecord{region.Outer}, region.Holes...) {
		area, err := loopSignedAreaBudget(budget, loop)
		if err != nil {
			return shellCancelCause(err)
		}
		if (li == 0 && !(area > 0)) || (li > 0 && !(area < 0)) {
			return fmt.Errorf(`%w: the shell's rim loop %d is walked against its material`, ErrDegenerate, li)
		}
	}
	if err := auditOffsetSectionBudget(budget, region, region); err != nil {
		return shellCancelCause(err)
	}
	return nil
}
