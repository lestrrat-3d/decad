package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/sectionaudit"

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
// (throughshell.RimRegions), and for each hole h of R the band between h and the
// hole of some Q that partners it — at a cap the hole of the eroded section
// of h's index, at a pierced wall the section of the tool through h dilated
// by t. A loop with no partner is SG7. Every rim region and band runs modify
// §5's audit (S8, S7, S9). A cap's rim carries the cavity's section
// displacement; a wall's rim, whose region states the cavity's levels and the
// receiver's cap levels as in-plane coordinates, carries the largest of that
// and every cavity level's displacement.
func throughCutRims(ctx context.Context, budget *proofbound.WorkBudget, bp brepPayload, tc throughCut, cavity brepPayload, eroded profileRecord, dilated []profileRecord, rm throughRemoval) (brepPayload, error) {
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
	if rm.Bottom {
		removed = append(removed, tc.caps.Bottom)
	}
	if rm.Top {
		removed = append(removed, tc.caps.Top)
	}
	removed = append(removed, rm.Walls...)
	rimTools := make([]throughshell.RimTool, len(tc.tools))
	for i, tool := range tc.tools {
		rimTools[i] = throughshell.RimTool{W0: tool.w0, W1: tool.w1,
			Loop0: tool.loop0, Loop1: tool.loop1, FrameEmbed: tool.frameEmb}
	}
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
		partner, reason, err := throughshell.NewRimPartner(ctx, throughshell.RimPartnerInput{
			FaceIndex: fi, Wall: rp.wall, Holes: r.region.Holes, Embed: rp.e,
			Caps: tc.caps, Joined: joined, Tools: rimTools, Dilated: dilated,
		})
		if err != nil {
			return brepPayload{}, err
		}
		if reason != "" {
			return brepPayload{}, throughRimError(r, reason)
		}
		var qOuters []loopRecord
		var bands []profileRecord
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
				var holeF loopRecord
				if !rp.wall {
					holeF = inF.Holes[qh]
				}
				hi, reason := partner(hole, holeF)
				if reason != "" {
					return brepPayload{}, throughRimError(r, reason)
				}
				if _, dup := partnered[hi]; dup {
					return brepPayload{}, throughRimError(r, "a cavity hole partners no hole of the removed face, or two")
				}
				partnered[hi] = struct{}{}
				band, err := offset2d.ReverseLoopRecordContext(ctx, hole)
				if err != nil {
					return brepPayload{}, err
				}
				bands = append(bands, profileRecord{Outer: band, Holes: []loopRecord{r.region.Holes[hi]}})
			}
		}
		if len(partnered) != len(r.region.Holes) {
			return brepPayload{}, throughRimError(r, "a hole of the removed face has no cavity hole to partner")
		}
		regions, reason, err := throughshell.RimRegions(ctx, budget, r.region.Outer, qOuters)
		if err != nil {
			return brepPayload{}, shellCancelCause(err)
		}
		if reason != "" {
			return brepPayload{}, throughRimError(r, reason)
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

// throughRimError is SG7, naming the removed face.
func throughRimError(r brepFace, what string) error {
	return fmt.Errorf(`%w: the shell's rim at %s does not partner the cavity's trace: %s (modify-general SG7)`, ErrUnsupported, r.role, what)
}

// auditThroughRim is modify §5's audit on one rim region or band: S8 (the
// outer loop walks counter-clockwise and every hole clockwise, each with a
// non-zero area), then S7 (no crossing or contact) and S9 (nesting) through
// the shared audit.
func auditThroughRim(budget *proofbound.WorkBudget, region profileRecord) error {
	for li, loop := range append([]loopRecord{region.Outer}, region.Holes...) {
		area, err := sectionaudit.LoopSignedArea(budget, loop)
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
