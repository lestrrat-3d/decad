package prismshell

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedbrep"
)

// StackInput states the receiver's levels and the selected caps beside its
// already audited side-opening sections.
type StackInput struct {
	Section                   Section
	Z0, Z1                    float64
	Z0Delta, Z1Delta          float64
	RemovedStart, RemovedEnd  bool
	Sense                     float64
	Thickness, ThicknessDelta float64
}

// shellLevel is one derived level of a shell's stack: from moved by by,
// carrying its source end's displacement, the thickness conversion and this
// float sum's own rounding (docs/shell-opening-design.md §3).
func shellLevel(from, delta, by, tDelta float64) stackedbrep.Level {
	to := from + by
	return stackedbrep.Level{Held: to, Delta: proofbound.AbsSumUpper(delta, tDelta, proofarith.AddRoundError(from, by, to))}
}

// Stack states BO2 (§4.1–§4.5). Inward the stack is the cap region P on
// [z0, z0 + t] where the start cap is kept, W over the cavity's height, and P
// on [z1 − t, z1] where the end cap is kept. Outward it is O on [z0 − t, z0],
// W on [z0, z1] and O on [z1, z1 + t]. Each interface exposes the cavity
// region once. A reflex end vertex is marked at both cavity levels.
func Stack(budget *proofbound.WorkBudget, in StackInput) (*stackedbrep.Engine, []stackedbrep.Level, float64, error) {
	sec := in.Section
	bottom := stackedbrep.Level{Held: in.Z0, Delta: in.Z0Delta}
	top := stackedbrep.Level{Held: in.Z1, Delta: in.Z1Delta}
	var levels []stackedbrep.Level
	var regions []momentinput.Profile
	wallAt := 0
	if in.Sense > 0 {
		levels = append(levels, bottom)
		if !in.RemovedStart {
			levels = append(levels, shellLevel(in.Z0, in.Z0Delta, in.Thickness, in.ThicknessDelta))
			regions = append(regions, sec.Caps)
			wallAt = 1
		}
		regions = append(regions, sec.Wall)
		if !in.RemovedEnd {
			levels = append(levels, shellLevel(in.Z1, in.Z1Delta, -in.Thickness, in.ThicknessDelta))
			regions = append(regions, sec.Caps)
		}
		levels = append(levels, top)
	} else {
		if !in.RemovedStart {
			levels = append(levels, shellLevel(in.Z0, in.Z0Delta, -in.Thickness, in.ThicknessDelta))
			regions = append(regions, sec.Caps)
			wallAt = 1
		}
		levels = append(levels, bottom)
		regions = append(regions, sec.Wall)
		levels = append(levels, top)
		if !in.RemovedEnd {
			levels = append(levels, shellLevel(in.Z1, in.Z1Delta, in.Thickness, in.ThicknessDelta))
			regions = append(regions, sec.Caps)
		}
	}
	for i := 1; i < len(levels); i++ {
		if !(levels[i].Held > levels[i-1].Held) {
			return nil, nil, 0, fmt.Errorf(`%w: a side opening's slab levels do not ascend (shell-opening SO3)`, decaderr.ErrDegenerate)
		}
	}
	geom, delta, err := stackEngine(budget, sec, levels, regions, wallAt)
	return geom, levels, delta, err
}

// stackEngine places the section loops and cavity interfaces on the stack.
func stackEngine(budget *proofbound.WorkBudget, sec Section, levels []stackedbrep.Level,
	regions []momentinput.Profile, wallAt int) (*stackedbrep.Engine, float64, error) {
	n := len(regions)
	held := make([]float64, len(levels))
	for i, l := range levels {
		held[i] = l.Held
	}
	geom := stackedbrep.NewEngine(held)
	for _, region := range regions {
		loop, err := geom.LoopOf(brepgeom.Profile{Outer: region.Outer, Holes: region.Holes}, budget)
		if err != nil {
			return nil, 0, err
		}
		geom.SlabLoops = append(geom.SlabLoops, []stackedbrep.Loop{loop})
	}
	if err := geom.AddFace(geom.SlabLoops[0][0], 0, false); err != nil {
		return nil, 0, err
	}
	if err := geom.AddFace(geom.SlabLoops[n-1][0], n, true); err != nil {
		return nil, 0, err
	}
	// The cavity region at each interface: the floor under the wall slab
	// faces up, the ceiling over it faces down.
	for _, iface := range []struct {
		level int
		up    bool
	}{{wallAt, true}, {wallAt + 1, false}} {
		if iface.level <= 0 || iface.level >= n {
			continue
		}
		loop, err := geom.LoopOf(brepgeom.Profile{Outer: sec.Cavity.Outer, Holes: sec.Cavity.Holes}, budget)
		if err != nil {
			return nil, 0, err
		}
		if err := geom.AddFace(loop, iface.level, iface.up); err != nil {
			return nil, 0, err
		}
	}
	for _, v := range sec.Corners {
		geom.Event(v, wallAt)
		geom.Event(v, wallAt+1)
	}
	geom.RecordJunctions(n)
	return geom, math.Max(sec.Delta, geom.Allow()), nil
}
