// Package analyticundercut maps recorded prism, revolve, and cup walls to
// their own-step topology faces for Verify's undercut survey.
package analyticundercut

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/cupwall"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolvesurvey"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/wallsurvey"
	"github.com/lestrrat-3d/r3"
)

type outcome = reportvocab.UndercutSurvey[*topo.Face]

func listVerdict(faces *[]*topo.Face, undecided *bool, f *topo.Face, verdict survey2d.PullVerdict, ok bool) bool {
	if !ok {
		return false
	}
	switch verdict {
	case survey2d.PullOpposes:
		*faces = append(*faces, f)
	case survey2d.PullUndecided:
		*undecided = true
	}
	return true
}

// Prism surveys a prism's faces against the pull, through the exact
// three-valued survey2d reader shared with Cup and capblendsurvey's
// receiver-wall loop: planar sides and caps carry one
// normal each, cylindrical sides sweep their walk's exact angular range. A
// straddling face sets undecided without discarding a face already proven to
// oppose (docs/verification-design.md §6).
//
// A surface result (pp.SurfaceResult) publishes no cap face at all
// (internal/prismrecord/build.go), so the cap loop below is skipped for one
// rather than probed and refused on a nil role lookup. On a solid a missing
// provenance.RoleCapStart or provenance.RoleCapEnd is a build defect and the
// existing total refusal must still catch it. The guard reads
// pp.SurfaceResult, never f == nil. Every
// wall input this function reads — the placed frame's exact directions, the
// walk's plane-local tangent or circular range, the caller's pull — is
// unchanged on a sheet, because §2.3 gives a surface result's walls the
// orientation the solid would have had: the identical outward normal, from
// the identical code. The survey therefore quantifies over exactly the
// sheet's own published faces, which are its walls.
func Prism(b *topo.Body, pp prismrecord.Payload, pull r3.Vec) reportvocab.UndercutSurvey[*topo.Face] {
	if _, ok := pull.Normalize(); !ok {
		return outcome{}
	}
	m, okM := survey2d.NewPlacedFrameMap(pp.Frame, pp.Xform)
	if !okM {
		return outcome{}
	}
	roles := topobuild.FacesByRole(b)
	loops, err := boundarywalk.SurveyLoops(nil, boundarywalk.Profile(pp.Profile))
	if err != nil {
		return outcome{}
	}
	faces := []*topo.Face{}
	undecided := false
	for li, loop := range loops {
		for _, w := range loop {
			f := roles[fmt.Sprintf("side(%d,%d)", li, w.Segs[0])]
			if f == nil {
				return outcome{}
			}
			verdict, ok := survey2d.WallNormalDecision(w, m, pull)
			if !listVerdict(&faces, &undecided, f, verdict, ok) {
				return outcome{}
			}
		}
	}
	if !pp.SurfaceResult {
		for _, cap := range []struct {
			role string
			sign float64
		}{{role: provenance.RoleCapStart, sign: -1}, {role: provenance.RoleCapEnd, sign: 1}} {
			f := roles[cap.role]
			if f == nil {
				return outcome{}
			}
			verdict, ok := survey2d.CapNormalDecision(m, pull, cap.sign)
			if !listVerdict(&faces, &undecided, f, verdict, ok) {
				return outcome{}
			}
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return outcome{Faces: faces, OK: true, Undecided: undecided}
}

// Revolve surveys a revolved body's faces against the pull: each
// wall's normal is n_ρ·radial(φ) + n_z·ŵ over the meridian range its walk
// sweeps and the azimuth range of the sweep — both exact.
func Revolve(b *topo.Body, rp revolveplan.Payload, pull r3.Vec) reportvocab.UndercutSurvey[*topo.Face] {
	if rp.SectionDelta != 0 {
		// The survey reads the recorded meridian's own tangents as exact, and a
		// displaced meridian's tangents only sit within its displacement of the
		// denoted ones: a planar wall recorded off a rounded miter tilts by an
		// ulp and would list as an undercut. Undecided, which reads Suspect
		// (docs/surface-intersection-design.md §7.2).
		return outcome{}
	}
	p, ok := pull.Normalize()
	if !ok {
		return outcome{}
	}
	bas := rp.Basis()
	pw := rp.Xform.ApplyDir(bas.W).Dot(p)
	c0 := rp.Xform.ApplyDir(bas.E0).Dot(p)
	c1 := rp.Xform.ApplyDir(bas.E1).Dot(p)
	roles := topobuild.FacesByRole(b)
	loops, err := wallsurvey.RevolveLoops(nil, rp.Profile, rp.Ax)
	if err != nil {
		return outcome{}
	}
	faces := []*topo.Face{}
	for _, decision := range revolvesurvey.UndercutRoles(
		loops, rp.Ax, pw, c0, c1, rp.Phi0, rp.Phi1, rp.Full, [2]string{provenance.RoleCapStart, provenance.RoleCapEnd},
	) {
		f := roles[decision.Role]
		if f == nil {
			return outcome{}
		}
		if decision.Opposes {
			faces = append(faces, f)
		}
	}
	return outcome{Faces: faces, OK: true}
}

// cupWalks resolves one cup region loop into its coalesced walks, as the cup
// wall builder does.
func cupWalks(loop sectionrecord.LoopRecord) ([]survey2d.SideWalk, error) {
	return cupWalksBudget(nil, loop)
}

func cupWalksBudget(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord) ([]survey2d.SideWalk, error) {
	loops, err := boundarywalk.SurveyLoopsBudget(budget, boundarywalk.Profile{Outer: loop})
	if err != nil {
		return nil, err
	}
	return loops[0], nil
}

// Cup surveys a cup's faces against the pull (docs/modify-design.md
// D2): the outer walls over EVERY loop of region O (role side(i,j)), the cavity
// walls over every loop of the reversed region C (role shellSide(i,j)) — each
// read by the same exact three-valued survey2d reader Prism uses, so a post
// wall is surveyed like any other — and the
// planar faces (the kept cap, the pocket floor, one rim per loop). The cavity
// being a pocket, the pocket floor (shellCap) and the rims face the open end
// and the kept cap (capStart) faces away from it, so their outward normals
// are ±N by which side of the outer floor the open end lies on.
func Cup(b *topo.Body, cp cupwall.View, pull r3.Vec) reportvocab.UndercutSurvey[*topo.Face] {
	if _, ok := pull.Normalize(); !ok {
		return outcome{}
	}
	base := cp.BasePrism()
	m, okM := survey2d.NewPlacedFrameMap(base.Frame, base.Xform)
	if !okM {
		return outcome{}
	}
	roles := topobuild.FacesByRole(b)

	faces := []*topo.Face{}
	undecided := false
	survey := func(loop sectionrecord.LoopRecord, role string) bool {
		walks, err := cupWalks(loop)
		if err != nil {
			return false
		}
		for _, w := range walks {
			f := roles[fmt.Sprintf(role, w.Segs[0])]
			if f == nil {
				return false
			}
			verdict, ok := survey2d.WallNormalDecision(w, m, pull)
			if !listVerdict(&faces, &undecided, f, verdict, ok) {
				return false
			}
		}
		return true
	}
	oLoops := append([]sectionrecord.LoopRecord{cp.Outer.Outer}, cp.Outer.Holes...)
	cLoops := append([]sectionrecord.LoopRecord{cp.Cavity.Outer}, cp.Cavity.Holes...)
	for i, loop := range oLoops {
		if !survey(loop, fmt.Sprintf("side(%d,%%d)", i)) {
			return outcome{}
		}
	}
	for i, loop := range cLoops {
		crev, err := offset2d.ReverseLoopRecord(loop)
		if err != nil {
			return outcome{}
		}
		if !survey(crev, fmt.Sprintf("shellSide(%d,%%d)", i)) {
			return outcome{}
		}
	}

	// The planar faces. sOpen is +1 when the open end lies above the outer
	// floor: the kept cap then faces −N and the pocket floor and every rim
	// face +N. All rims share the open-end plane, so each has the same normal.
	sOpen := -1.0
	if cp.ZOpen > cp.ZOuter {
		sOpen = 1
	}
	caps := []struct {
		role string
		sign float64
	}{
		{role: provenance.RoleCapStart, sign: -sOpen},
		{role: cupwall.RoleShellCap, sign: sOpen},
	}
	for i := range oLoops {
		caps = append(caps, struct {
			role string
			sign float64
		}{role: fmt.Sprintf("rim(%d)", i), sign: sOpen})
	}
	for _, c := range caps {
		f := roles[c.role]
		if f == nil {
			return outcome{}
		}
		verdict, ok := survey2d.CapNormalDecision(m, pull, c.sign)
		if !listVerdict(&faces, &undecided, f, verdict, ok) {
			return outcome{}
		}
	}
	if undecided && len(faces) == 0 {
		// Keep an entirely undecided result distinct from a proven all-clear.
		faces = nil
	}
	return outcome{Faces: faces, OK: true, Undecided: undecided}
}
