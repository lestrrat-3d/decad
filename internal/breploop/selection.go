package breploop

import (
	"fmt"
	"math"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// SelectionInput binds original public-handle order to immutable topology nodes.
// Describe renders the local ordinal, retaining the caller's diagnostic text.
type SelectionInput struct {
	Edges    []*topo.Edge
	Describe func(local, original int) string
	Asym     *AsymmetricInput
}

// AsymmetricInput keeps live reference nodes and the original receiver producer.
type AsymmetricInput struct {
	Receiver          *topo.Body
	References        []*topo.Face
	RecordRoles       []string
	D, DDelta         float64
	Other, OtherDelta float64
}

// Selection is an ordered view of one state's matches and work budget.
type Selection struct {
	State    *State
	Input    SelectionInput
	Ordinals []int
}

// Bind constructs the full ordered view without matching or charging work.
func Bind(state *State, input SelectionInput) Selection {
	ordinals := make([]int, len(input.Edges))
	for i := range ordinals {
		ordinals[i] = i
	}
	return Selection{State: state, Input: input, Ordinals: ordinals}
}

// View clones a subset's original ordinals while sharing state and budget.
func (s Selection) View(ordinals []int) Selection {
	s.Ordinals = slices.Clone(ordinals)
	return s
}

func (s Selection) Len() int { return len(s.Ordinals) }

func (s Selection) Match(i int) int { return s.State.Matched[s.Ordinals[i]] }

func (s Selection) describe(i int) string { return s.Input.Describe(i, s.Ordinals[i]) }

func (s Selection) text() string {
	parts := make([]string, s.Len())
	for i := range parts {
		parts[i] = s.describe(i)
	}
	return strings.Join(parts, "; ")
}

func (s Selection) refuse(reason string) error {
	return s.State.RefuseWithSelection("SL1", reason, s.text())
}

// MatchEdges records each original selected edge's record index in original order.
func (s Selection) MatchEdges() error {
	refView := s.State.Payload.RefView()
	const tol = 1e-6
	s.State.Matched = make([]int, len(s.Input.Edges))
	for i, edge := range s.Input.Edges {
		s.State.Matched[i] = -1
		for ei, pair := range s.State.Topology.Edges {
			if err := survey2d.WallBudgetStep(s.State.Budget); err != nil {
				return err
			}
			if !edgeMatches(refView, s.State.Topology.Uses[pair[0]], edge, tol) {
				continue
			}
			if s.State.Matched[i] != -1 {
				s.State.Matched[i] = -2
				break
			}
			s.State.Matched[i] = ei
		}
	}
	return nil
}

func edgeMatches(refView prismrecord.Payload, owner brepgeom.Use, edge *topo.Edge, tol float64) bool {
	if edge.Start == nil || edge.End == nil {
		return false
	}
	lift := func(c [3]float64) r3.Vec { return refView.Point(c[0], c[1], c[2]) }
	key := owner.Key
	switch curve := edge.Curve.(type) {
	case surfacegeom.Line3:
		if key.Circular {
			return false
		}
	case surfacegeom.Arc3:
		if !key.Circular || key.Closed || lift(key.C).Sub(curve.Center).Len() > tol {
			return false
		}
	case surfacegeom.Circle3:
		if !key.Closed || lift(key.C).Sub(curve.Center).Len() > tol || math.Abs(curve.Radius.Base()-key.Radius) > tol {
			return false
		}
		var unit [3]float64
		unit[key.Axis] = 1
		axis := refView.Dir(unit[0], unit[1], unit[2])
		return axis.Cross(curve.Axis).Len() <= tol
	default:
		return false
	}
	return brepgeom.MatchEndpoints(edge.Start.Position, edge.End.Position,
		lift(owner.DirFrom), lift(owner.DirTo), tol)
}

// SingleStraightEdges reports route E's independent axis-aligned edge class.
func (s Selection) SingleStraightEdges() bool {
	seen := map[[3]float64]struct{}{}
	for i := range s.Len() {
		ei := s.Match(i)
		if ei < 0 {
			return false
		}
		owner := s.State.Topology.Uses[s.State.Topology.Edges[ei][0]]
		if owner.Key.Circular {
			return false
		}
		differ := 0
		for axis := range 3 {
			if owner.DirFrom[axis] != owner.DirTo[axis] {
				differ++
			}
		}
		if differ != 1 {
			return false
		}
		for _, v := range [2][3]float64{owner.DirFrom, owner.DirTo} {
			if _, ok := seen[v]; ok {
				return false
			}
			seen[v] = struct{}{}
		}
	}
	return true
}

// AdmitLoops keeps LB1/LB2's first refusal and first encountered complete loop.
func (s Selection) AdmitLoops() ([]Sel, error) {
	type faceLoop struct{ face, loop int }
	selected := map[faceLoop]map[int]struct{}{}
	loopsOf := make([][]faceLoop, s.Len())
	for i := range s.Len() {
		ei := s.Match(i)
		switch ei {
		case -1:
			return nil, s.refuse(fmt.Sprintf(`%s matches no edge of the receiver's face record`, s.describe(i)))
		case -2:
			return nil, s.refuse(fmt.Sprintf(`%s matches more than one edge of the receiver's face record`, s.describe(i)))
		}
		for _, ui := range s.State.Topology.Edges[ei] {
			u := s.State.Topology.Uses[ui]
			if u.Part != brepgeom.LoopSeg {
				continue
			}
			fl := faceLoop{u.Face, u.Loop}
			loopsOf[i] = append(loopsOf[i], fl)
			if selected[fl] == nil {
				selected[fl] = map[int]struct{}{}
			}
			selected[fl][u.Seg] = struct{}{}
		}
	}
	complete := func(fl faceLoop) bool {
		return len(selected[fl]) == len(s.State.Topology.Planar[fl.face][fl.loop])
	}
	chosen := map[faceLoop]struct{}{}
	var out []Sel
	for i := range s.Len() {
		var covering []faceLoop
		for _, fl := range loopsOf[i] {
			if complete(fl) {
				covering = append(covering, fl)
			}
		}
		switch {
		case len(loopsOf[i]) == 0:
			return nil, s.refuse(fmt.Sprintf(`%s lies on no loop of a planar face, so it belongs to no complete loop; a %s of loops mixed with single edges needs a corner patch in no reference frame`, s.describe(i), s.State.Kind))
		case len(covering) == 0:
			return nil, s.refuse(fmt.Sprintf(`the selection covers only part of a loop of brep face %s; every geometric edge of a complete loop must be selected`, s.State.Payload.Faces[loopsOf[i][0].face].Role))
		case len(covering) > 1:
			return nil, s.refuse(fmt.Sprintf(`%s lies on two selected complete loops, of brep faces %s and %s, which share it`,
				s.describe(i), s.State.Payload.Faces[covering[0].face].Role, s.State.Payload.Faces[covering[1].face].Role))
		}
		fl := covering[0]
		if _, ok := chosen[fl]; ok {
			continue
		}
		chosen[fl] = struct{}{}
		out = append(out, Sel{Face: fl.face, Loop: fl.loop})
	}
	return out, nil
}

// PartialSelectedLoop finds one planar loop containing this selection as a proper subset.
func (s Selection) PartialSelectedLoop() (Sel, []bool, bool) {
	if s.Len() == 0 || s.Match(0) < 0 {
		return Sel{}, nil, false
	}
	for _, firstUse := range s.State.Topology.Edges[s.Match(0)] {
		u := s.State.Topology.Uses[firstUse]
		if u.Part != brepgeom.LoopSeg || !s.State.Payload.Faces[u.Face].Planar() {
			continue
		}
		selected := make([]bool, len(s.State.Topology.Planar[u.Face][u.Loop]))
		valid := true
		for i := range s.Len() {
			ei := s.Match(i)
			if ei < 0 {
				valid = false
				break
			}
			found := false
			for _, ui := range s.State.Topology.Edges[ei] {
				v := s.State.Topology.Uses[ui]
				if v.Part == brepgeom.LoopSeg && v.Face == u.Face && v.Loop == u.Loop {
					selected[v.Seg] = true
					found = true
				}
			}
			if !found {
				valid = false
				break
			}
		}
		if !valid || s.Len() >= len(selected) {
			continue
		}
		return Sel{Face: u.Face, Loop: u.Loop}, selected, true
	}
	return Sel{}, nil, false
}

// SplitResult separates complete loops from the original route E edge ordinals.
type SplitResult struct {
	Loops            []Sel
	StraightOrdinals []int
	Mixed            bool
}

// SplitVertexBlend tries reference axes in order, then complete-loop coverage.
func (s Selection) SplitVertexBlend() (SplitResult, error) {
	for axis := range 3 {
		var singleOrdinals, loopOrdinals []int
		for i, ei := range s.State.Matched {
			if ei < 0 {
				return SplitResult{}, nil
			}
			owner := s.State.Topology.Uses[s.State.Topology.Edges[ei][0]]
			along := !owner.Key.Circular && owner.DirFrom[axis] != owner.DirTo[axis]
			for other := range 3 {
				if other != axis && owner.DirFrom[other] != owner.DirTo[other] {
					along = false
				}
			}
			if along {
				singleOrdinals = append(singleOrdinals, i)
			} else {
				loopOrdinals = append(loopOrdinals, i)
			}
		}
		if len(singleOrdinals) == 0 || len(loopOrdinals) == 0 {
			continue
		}
		if !s.View(singleOrdinals).SingleStraightEdges() {
			continue
		}
		loops, err := s.View(loopOrdinals).AdmitLoops()
		if err == nil {
			return SplitResult{Loops: loops, StraightOrdinals: singleOrdinals, Mixed: true}, nil
		}
	}

	type faceLoop struct{ face, loop int }
	selected := map[faceLoop]map[int]struct{}{}
	for _, ei := range s.State.Matched {
		if ei < 0 {
			return SplitResult{}, nil
		}
		for _, ui := range s.State.Topology.Edges[ei] {
			u := s.State.Topology.Uses[ui]
			if u.Part != brepgeom.LoopSeg {
				continue
			}
			fl := faceLoop{u.Face, u.Loop}
			if selected[fl] == nil {
				selected[fl] = map[int]struct{}{}
			}
			selected[fl][u.Seg] = struct{}{}
		}
	}
	var loopOrdinals, singleOrdinals []int
	for i, ei := range s.State.Matched {
		covering := 0
		for _, ui := range s.State.Topology.Edges[ei] {
			u := s.State.Topology.Uses[ui]
			if u.Part == brepgeom.LoopSeg &&
				len(selected[faceLoop{u.Face, u.Loop}]) == len(s.State.Topology.Planar[u.Face][u.Loop]) {
				covering++
			}
		}
		if covering > 1 {
			return SplitResult{}, s.State.Refuse("SL1", fmt.Sprintf(`%s lies on two selected complete loops`, s.Input.Describe(i, i)))
		}
		if covering == 1 {
			loopOrdinals = append(loopOrdinals, i)
			continue
		}
		singleOrdinals = append(singleOrdinals, i)
	}
	if len(loopOrdinals) == 0 || len(singleOrdinals) == 0 {
		return SplitResult{}, nil
	}
	if !s.View(singleOrdinals).SingleStraightEdges() {
		return SplitResult{}, s.State.Refuse("SL1", `the edges outside the complete loops are not independent straight edges along reference axes`)
	}
	loops, err := s.View(loopOrdinals).AdmitLoops()
	if err != nil {
		return SplitResult{}, err
	}
	return SplitResult{Loops: loops, StraightOrdinals: singleOrdinals, Mixed: true}, nil
}

// AssignLoopSetbacks preserves original-order reference assignment and SX refusal order.
func (s Selection) AssignLoopSetbacks(sels []Sel) error {
	for i := range sels {
		sels[i].Setback = *s.State.Setback
	}
	if s.Input.Asym == nil {
		return nil
	}
	asym := s.Input.Asym
	selected := make(map[int]*topo.Face, len(s.State.Matched))
	for i, ei := range s.State.Matched {
		selected[ei] = asym.References[i]
	}
	seen := map[[2]int]capblend.Setback{}
	for i := range sels {
		sel := &sels[i]
		assignment := -1
		for segment, beside := range sel.Beside {
			ui := s.State.LoopUse[[3]int{sel.Face, sel.Loop, segment}]
			ref := selected[s.State.Topology.EdgeOf[ui]]
			capFace := breprecord.MatchRecordFace(asym.Receiver, ref, asym.RecordRoles, sel.Face)
			sideFace := breprecord.MatchRecordFace(asym.Receiver, ref, asym.RecordRoles,
				s.State.Topology.Uses[beside.Use].Face)
			if capFace == sideFace {
				return s.State.RefuseAsymmetricLoop("SX16", fmt.Sprintf(`the reference for brep face %s's loop segment %d has no single adjacent record-face identity`,
					s.State.Payload.Faces[sel.Face].Role, segment))
			}
			choice := 0
			if sideFace {
				choice = 1
			}
			if assignment >= 0 && assignment != choice {
				return s.State.RefuseAsymmetricLoop("SX4", fmt.Sprintf(`brep face %s's loop mixes cap and side reference assignments`,
					s.State.Payload.Faces[sel.Face].Role))
			}
			assignment = choice
		}
		if assignment < 0 {
			return s.State.RefuseAsymmetricLoop("SX16", fmt.Sprintf(`brep face %s's loop has no edges to assign a reference face`,
				s.State.Payload.Faces[sel.Face].Role))
		}
		if assignment == 0 {
			sel.Setback = capblend.Setback{Dc: asym.D, DcDelta: asym.DDelta,
				Ds: asym.Other, DsDelta: asym.OtherDelta}
		} else {
			sel.Setback = capblend.Setback{Dc: asym.Other, DcDelta: asym.OtherDelta,
				Ds: asym.D, DsDelta: asym.DDelta}
		}
		f := s.State.Payload.Faces[sel.Face]
		band := breprecord.LoopBand{Sigma: sel.Sigma}
		side := 0
		if band.MatSign(f) > 0 {
			side = 1
		}
		key := [2]int{sel.Face, side}
		if old, ok := seen[key]; ok && old != sel.Setback {
			return s.State.RefuseAsymmetricLoop("SX4", fmt.Sprintf(`brep face %s's loops on one material side use different cap and side setbacks`, f.Role))
		}
		seen[key] = sel.Setback
	}
	return nil
}
