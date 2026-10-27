package loftrecord

import (
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// PairingReuse carries the record-only work of one FromProfilesReusing build
// to a later build of the same two records on other planes, such as the
// free-form rim chamfer's second cutter, which lofts the first one's two rings
// at the other cap.
//
// That work is the two area falsifiers of FromProfiles and pairPayload, Build's
// record-only half. Each is a deterministic function of the two split profile
// records, the two sketch areas, the alignment and the two work counters, and
// FromProfiles opens both counters fresh and raises them to limits read from
// the first record alone. So two builds whose records, areas and alignment are
// identical run that work on identical inputs: what succeeded for the first
// succeeds for the second with the same results. The one exception is
// pairPayload's S5 gate, the only step that reads the planes. The steps before
// it passed on the same inputs, so a build that reads reuse runs that gate on
// its own planes before it takes the held pairing, and refuses where a fresh
// build refuses. The counters are not read again after pairPayload, so the
// build that skips the work has no use for the charges it skips.
//
// The records are compared by sectionrecord.IdenticalRecord, the sketch areas
// by their float bits, and the alignment exactly, nil and empty apart. Any
// difference builds afresh. reuse keeps its own copies of the records and the
// pairing and hands each build a fresh copy of the pairing, so no build sees
// another one's writes.
//
// A PairingReuse is not safe for concurrent use. Its zero value holds nothing.
type PairingReuse struct {
	held   *heldPairing
	reused int
}

// heldPairing is one build's record-only results, under the inputs they were
// computed from.
type heldPairing struct {
	profile0, profile1 momentinput.Profile
	area0, area1       float64
	alignment          []int
	recordArea         [2]float64
	paired             recordPairing
}

// Reused reports how many builds read r's held results.
func (r *PairingReuse) Reused() int { return r.reused }

// lookup returns the held results for exactly these inputs, or nil.
func (r *PairingReuse) lookup(profile0, profile1 momentinput.Profile, area0, area1 float64,
	alignment []int) *heldPairing {
	if r == nil || r.held == nil {
		return nil
	}
	h := r.held
	if math.Float64bits(h.area0) != math.Float64bits(area0) || math.Float64bits(h.area1) != math.Float64bits(area1) ||
		(h.alignment == nil) != (alignment == nil) || !slices.Equal(h.alignment, alignment) ||
		!sectionrecord.IdenticalRecord(h.profile0, profile0) || !sectionrecord.IdenticalRecord(h.profile1, profile1) {
		return nil
	}
	return h
}

// hold keeps next under copies of its inputs. A nil r or next keeps nothing.
func (r *PairingReuse) hold(next *heldPairing, profile0, profile1 momentinput.Profile, area0, area1 float64,
	alignment []int) {
	if r == nil || next == nil {
		return
	}
	next.profile0, next.profile1 = cloneProfile(profile0), cloneProfile(profile1)
	next.area0, next.area1 = area0, area1
	next.alignment = slices.Clone(alignment)
	r.held = next
}

func cloneProfile(p momentinput.Profile) momentinput.Profile {
	out := momentinput.Profile{Outer: sectionrecord.CloneLoopRecord(p.Outer)}
	if p.Holes != nil {
		out.Holes = make([]momentinput.LoopRecord, len(p.Holes))
		for i, hole := range p.Holes {
			out.Holes[i] = sectionrecord.CloneLoopRecord(hole)
		}
	}
	return out
}

// clone copies every slice of the pairing.
func (p recordPairing) clone() recordPairing {
	out := p
	out.pairs = make([]loftmesh.LoopPair, len(p.pairs))
	for i, pair := range p.pairs {
		out.pairs[i] = loftmesh.LoopPair{
			V: slices.Clone(pair.V), W: slices.Clone(pair.W),
			ArcUpperV: slices.Clone(pair.ArcUpperV), ArcUpperW: slices.Clone(pair.ArcUpperW),
			MatchedDelta:   slices.Clone(pair.MatchedDelta),
			TangentEnergyV: slices.Clone(pair.TangentEnergyV), TangentEnergyW: slices.Clone(pair.TangentEnergyW),
			Segment: slices.Clone(pair.Segment),
			Faceted: slices.Clone(pair.Faceted),
		}
	}
	return out
}
