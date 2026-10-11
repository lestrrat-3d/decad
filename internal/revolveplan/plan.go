// Package revolveplan resolves a recorded revolve section and selects its
// meridian and angular mesh counts before any facets are assembled.
package revolveplan

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/revolvesampling"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ResolveInput carries the record fields needed before a chord count exists.
type ResolveInput struct {
	Lift        revolvemesh.RevolveLift
	Loops       []sectionrecord.LoopRecord
	Charge      revolveaxis.WalkCharge
	SnapTol     float64
	RadialLower float64
	Transform   r3.Transform
	// WorkLimit applies only when a composite Sweep replays its certified
	// fitted-spline profile. Zero preserves Revolve's ordinary limit.
	WorkLimit uint64
}

// Resolution holds the recorded walks and count-independent coordinate bounds.
type Resolution struct {
	Basis       revolvemesh.RevolveBasis
	Ideal       revolvemesh.RevolveBasis3Iv
	Loops       []sectionrecord.LoopRecord
	Resolved    []revolveaxis.ResolvedWalks
	Junctions   [][]revolvemesh.RevMeridian
	RhoMax      float64
	CoordMax    float64
	SamplePrior float64
	DeltaCPrior float64
	DeltaRPrior float64
	RadialLower float64
}

type walkView []revolveaxis.ResolvedWalks

func (loops walkView) Len() int                        { return len(loops) }
func (loops walkView) Walks(i int) []survey2d.SideWalk { return loops[i].Walks }

// Resolve reads the same walks as the revolve builder and reserves coordinate
// error before choosing a meridian or angular chord count.
func Resolve(ctx context.Context, input ResolveInput) (*Resolution, error) {
	lift := input.Lift
	ideal, ok := revolvemesh.IdealBasis(lift.Frame, lift.AU, lift.AV, lift.DU, lift.DV)
	if !ok {
		return nil, fmt.Errorf(`%w: this revolve's axis basis holds a coordinate that cannot be enclosed, so the mesh can state no construction bound`, decaderr.ErrUnsupported)
	}
	work := freeform.NewFreeformWork()
	if input.WorkLimit > 0 {
		work.RaiseLimit(input.WorkLimit)
	}
	resolved := make([]revolveaxis.ResolvedWalks, len(input.Loops))
	junctions := make([][]revolvemesh.RevMeridian, len(input.Loops))
	junctionGap := 0.0
	for li, loop := range input.Loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		r, err := revolveaxis.ResolveLoopWithFreeform(ctx, loop, work, "revolve tessellation",
			input.Charge, input.SnapTol, input.RadialLower)
		if err != nil {
			return nil, err
		}
		js, gap, err := revolvesampling.MeridianJunctions(lift, r)
		if err != nil {
			return nil, err
		}
		junctionGap = math.Max(junctionGap, gap)
		resolved[li], junctions[li] = r, js
	}
	if err := revolvesampling.RequireAxisIncidence(resolved, junctions); err != nil {
		return nil, err
	}
	rhoMax, zAbsMax, err := revolveproof.Extents(walkView(resolved))
	if err != nil {
		return nil, err
	}
	basis := lift.Basis()
	coordMax := revolvemesh.RevolveCoordMax(basis, zAbsMax, rhoMax)
	stationPrior := proofbound.ProductUpper(revolvemesh.RevolveStationRoundUlps,
		proofbound.UlpOf(math.Max(math.Max(zAbsMax, rhoMax), 1)))
	samplePrior := math.Max(junctionGap, stationPrior)
	if proofbound.IsNonFinite(coordMax) || proofbound.IsNonFinite(samplePrior) {
		return nil, fmt.Errorf(`%w: this revolve's coordinate envelope is not finite, so no chord budget can be reserved against it`, decaderr.ErrUnsupported)
	}
	deltaCPrior := revolvemesh.RevolveConstructionPrior(basis, samplePrior, rhoMax, coordMax)
	deltaRPrior := proofbound.RigidRoundAllow(proofbound.AbsSumUpper(coordMax, deltaCPrior),
		proofbound.VecMaxAbs(input.Transform.Translation()))
	return &Resolution{
		Basis: basis, Ideal: ideal, Loops: input.Loops, Resolved: resolved, Junctions: junctions,
		RhoMax: rhoMax, CoordMax: coordMax, SamplePrior: samplePrior,
		DeltaCPrior: deltaCPrior, DeltaRPrior: deltaRPrior,
		RadialLower: input.RadialLower,
	}, nil
}

// Counts are the current meridian and angular chording choices. Refinement
// increases only one of them after a mesh audit asks for a retry.
type Counts struct {
	Meridian       [][]int
	Sagittas       [][]float64
	Freeform       [][]*freeform.FreeformChain
	Angular        int
	MeridianTarget float64
	MeridianDelta  float64
	AngularDelta   float64
	Sweep          float64
	RhoMax         float64
}

// CountInput carries the tolerance and angle fields needed to choose counts.
type CountInput struct {
	Resolution   *Resolution
	Chord        float64
	SectionDelta float64
	Phi0, Phi1   float64
	Full         bool
	// MeridianFloor shares profile stations with adjacent sweep spans.
	// Nil keeps Revolve's ordinary minimum counts.
	MeridianFloor [][]int
	// FreeformTarget selects the same certified dyadic station chain in
	// adjacent composite Sweep spans. Zero uses this Revolve's own target.
	FreeformTarget float64
	// WorkLimit applies to the same composite Sweep profile replay as
	// ResolveInput.WorkLimit. Zero keeps Revolve's ordinary limit.
	WorkLimit uint64
}

// PlanCounts reserves section and coordinate displacement, then chords each
// circular meridian walk and the global angular sequence in that order.
func PlanCounts(input CountInput) (Counts, error) {
	res := input.Resolution
	budgetChord := input.Chord
	if input.SectionDelta > 0 {
		budgetChord = freeform.DownRound(freeform.DownRound(input.Chord - input.SectionDelta))
		if budgetChord <= 0 {
			requested, displacement := units.Millimeters(input.Chord), units.Millimeters(input.SectionDelta)
			return Counts{}, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s; retry with a tolerance greater than %s`, decaderr.ErrUnsupported, requested, displacement, displacement)
		}
	}
	available, err := revolvemesh.RevolveBudget(budgetChord, res.DeltaCPrior, res.DeltaRPrior)
	if err != nil {
		return Counts{}, err
	}
	meridian := freeform.DownRound(available / 2)
	counts := Counts{
		Meridian: make([][]int, len(res.Resolved)), Sagittas: make([][]float64, len(res.Resolved)),
		Freeform: make([][]*freeform.FreeformChain, len(res.Resolved)),
		RhoMax:   res.RhoMax, MeridianTarget: meridian,
	}
	freeformWork := freeform.NewFreeformWork()
	if input.WorkLimit > 0 {
		freeformWork.RaiseLimit(input.WorkLimit)
	}
	if input.MeridianFloor != nil && len(input.MeridianFloor) != len(res.Resolved) {
		return Counts{}, fmt.Errorf(`%w: the shared meridian count plan has a different loop count`, decaderr.ErrUnsupported)
	}
	for li, r := range res.Resolved {
		if input.MeridianFloor != nil && len(input.MeridianFloor[li]) != len(r.Walks) {
			return Counts{}, fmt.Errorf(`%w: the shared meridian count plan has a different walk count`, decaderr.ErrUnsupported)
		}
		counts.Meridian[li] = make([]int, len(r.Walks))
		counts.Sagittas[li] = make([]float64, len(r.Walks))
		counts.Freeform[li] = make([]*freeform.FreeformChain, len(r.Walks))
		for k, w := range r.Walks {
			counts.Meridian[li][k] = 1
			if w.Kind == survey2d.WalkFreeform {
				target := meridian
				if input.FreeformTarget > 0 {
					if input.FreeformTarget > meridian {
						return Counts{}, fmt.Errorf(`%w: shared free-form stations exceed this Revolve's meridian budget`, decaderr.ErrUnsupported)
					}
					target = input.FreeformTarget
				}
				chain, err := freeform.ChainStations(w.Spans, target, freeformWork)
				if err != nil {
					return Counts{}, err
				}
				if input.MeridianFloor != nil && input.MeridianFloor[li][k] > 0 &&
					len(chain.Stations) != input.MeridianFloor[li][k] {
					return Counts{}, fmt.Errorf(`%w: shared free-form stations have a different meridian count`, decaderr.ErrUnsupported)
				}
				if chain.Sagitta >= res.RadialLower {
					return Counts{}, fmt.Errorf(`%w: a free-form meridian's chord tube reaches the revolve axis`, decaderr.ErrUnsupported)
				}
				counts.Freeform[li][k] = &chain
				counts.Meridian[li][k] = len(chain.Stations)
				counts.Sagittas[li][k] = chain.Sagitta
				counts.MeridianDelta = math.Max(counts.MeridianDelta, chain.Sagitta)
				continue
			}
			if !w.IsCircular() {
				if input.MeridianFloor != nil && input.MeridianFloor[li][k] > 1 {
					return Counts{}, fmt.Errorf(`%w: a straight meridian cannot take extra chord stations`, decaderr.ErrUnsupported)
				}
				continue
			}
			n, sag, err := tessellation.ChordCount(w.SegmentWalk, meridian, revolvesampling.MeridianMin(w.SegmentWalk))
			if err != nil {
				return Counts{}, err
			}
			if input.MeridianFloor != nil && input.MeridianFloor[li][k] > n {
				n = input.MeridianFloor[li][k]
				if n > freeform.MaxChordsPerWalk {
					return Counts{}, freeform.ErrTooManyChords
				}
				sag = tessellation.ChordSagitta(w.Radius, math.Abs(w.Th1-w.Th0), n)
			}
			counts.Meridian[li][k], counts.Sagittas[li][k] = n, sag
			counts.MeridianDelta = math.Max(counts.MeridianDelta, sag)
		}
	}
	counts.Sweep = math.Abs(input.Phi1 - input.Phi0)
	angular := freeform.DownRound(available - counts.MeridianDelta)
	if angular <= 0 || proofbound.IsNonFinite(angular) {
		return Counts{}, fmt.Errorf(`%w: this revolve's meridian chording spends the whole chord budget its tolerance left, so no angular count remains; retry with a coarser tolerance`, decaderr.ErrUnsupported)
	}
	angularWalk := survey2d.SegmentWalk{Radius: res.RhoMax, Th1: counts.Sweep, Closed: input.Full}
	counts.Angular, counts.AngularDelta, err = tessellation.ChordCount(angularWalk, angular, tessellation.ChordWalkMin(angularWalk))
	if err != nil {
		return Counts{}, err
	}
	return counts, nil
}

// Refinement identifies one meridian walk, or the global angular count when
// Loop is negative.
type Refinement struct {
	Loop, Walk int
}

// Refine increments the requested count and recomputes its sagitta bound.
func (counts *Counts) Refine(resolved []revolveaxis.ResolvedWalks, request Refinement) error {
	if request.Loop < 0 {
		n := counts.Angular + 1
		if n > freeform.MaxChordsPerWalk {
			return freeform.ErrTooManyChords
		}
		counts.Angular = n
		counts.AngularDelta = tessellation.ChordSagitta(counts.RhoMax, counts.Sweep, n)
		return nil
	}
	w := resolved[request.Loop].Walks[request.Walk]
	if !w.IsCircular() {
		return fmt.Errorf(`%w: a straight revolve generator carries no meridian chording to refine`, decaderr.ErrUnsupported)
	}
	n := counts.Meridian[request.Loop][request.Walk] + 1
	if n > freeform.MaxChordsPerWalk {
		return freeform.ErrTooManyChords
	}
	counts.Meridian[request.Loop][request.Walk] = n
	counts.Sagittas[request.Loop][request.Walk] = tessellation.ChordSagitta(w.Radius, math.Abs(w.Th1-w.Th0), n)
	counts.MeridianDelta = 0
	for li := range counts.Sagittas {
		for _, sag := range counts.Sagittas[li] {
			counts.MeridianDelta = math.Max(counts.MeridianDelta, sag)
		}
	}
	return nil
}
