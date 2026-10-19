package loftcontact

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
)

// SourcePartition is a transient source certificate, never a body payload.
// It describes open side patches and cap incidence, not a held output shell.
type SourcePartition struct {
	Carrier  int
	Crossing [2]int
	Seams    [2]LineSeam
	// -1 hides the whole Loft wall, +1 exposes it, 0 splits it at a seam.
	Walls        []int8
	Junction     [2][2]sectionrecord.Point2
	CapOwners    [2][3]int // prism, Loft, prism-owned overlap cells
	CapLevels    [3]*big.Rat
	TopWallJoins int
	Prepared     [2]PreparedCap
	SourceVisits uint64
}

type capSpan struct{ lo, hi *big.Rat }

// Each reconstructed entity gets exactly one source, direction, and cap level.
// This map is local to one Sketch arrangement and is never retained by a body.
type CapOrigin struct {
	Operand, Loop, Segment int
	Reversed               bool
	Cap                    int
	Level                  *big.Rat
	Line                   sectionrecord.LineSeg
}

type fragmentKey struct {
	operand, segment int
	lo, hi           string
}

type fragmentUse struct {
	cell     int
	reversed bool
}

// CapProof is the transient result of PartitionCap.
type CapProof struct {
	Junction    [2]sectionrecord.Point2
	Level       *big.Rat
	Owners      [3]int
	CarrierUses int
}

// PreparedCap holds one private Sketch cap scene, its source tags and its raw
// paired cells.
type PreparedCap struct {
	Scene    *sketch.Sketch
	Tags     map[sketch.Entity]prismcells.Origin
	Raw      []*sketch.Profile
	CapIndex int
	Level    *big.Rat
}

func spanEqual(budget *proofbound.WorkBudget, got, want []capSpan,
	checked ...*Arithmetic) (bool, error) {
	arithmetic := ReplayArithmetic(checked)
	if len(got) != len(want) {
		return false, nil
	}
	for _, spans := range [][]capSpan{got, want} {
		for i := 1; i < len(spans); i++ {
			for j := i; j > 0; j-- {
				if err := budget.Step(); err != nil {
					return false, err
				}
				cmp := arithmetic.Cmp(spans[j-1].lo, spans[j].lo)
				if !arithmetic.OK {
					return false, nil
				}
				if cmp <= 0 {
					break
				}
				spans[j-1], spans[j] = spans[j], spans[j-1]
			}
		}
	}
	for i := range got {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if arithmetic.Cmp(got[i].lo, want[i].lo) != 0 ||
			arithmetic.Cmp(got[i].hi, want[i].hi) != 0 || !arithmetic.OK {
			return false, nil
		}
	}
	return true, nil
}

// SceneOrigins maps every entity of scene to one source line at capIndex.
func SceneOrigins(budget *proofbound.WorkBudget, scene *sketch.Sketch,
	tags map[sketch.Entity]prismcells.Origin, prismLines, loftLines []sectionrecord.LineSeg,
	capIndex int, level *big.Rat,
) (map[sketch.Entity]CapOrigin, bool, error) {
	if len(tags) != len(prismLines)+len(loftLines) {
		return nil, false, nil
	}
	entities := scene.Entities()
	if len(entities) != len(tags) {
		return nil, false, nil
	}
	seen := [2][]bool{make([]bool, len(prismLines)), make([]bool, len(loftLines))}
	mapped := make(map[sketch.Entity]CapOrigin, len(tags))
	for _, entity := range entities {
		if err := budget.Step(); err != nil {
			return nil, false, err
		}
		origin, ok := tags[entity]
		if !ok || origin.Region != 0 || origin.Hole != -1 || origin.AuthoredReversed {
			return nil, false, nil
		}
		op := 0
		lines := prismLines
		if origin.IsB {
			op, lines = 1, loftLines
		}
		if origin.Segment < 0 || origin.Segment >= len(lines) || seen[op][origin.Segment] {
			return nil, false, nil
		}
		line, ok := entity.(*sketch.Line)
		if !ok || line.Start.X() != lines[origin.Segment].Start.U ||
			line.Start.Y() != lines[origin.Segment].Start.V ||
			line.End.X() != lines[origin.Segment].End.U ||
			line.End.Y() != lines[origin.Segment].End.V {
			return nil, false, nil
		}
		seen[op][origin.Segment] = true
		mapped[entity] = CapOrigin{
			Operand: op, Loop: -1, Segment: origin.Segment, Reversed: origin.AuthoredReversed,
			Cap: capIndex, Level: new(big.Rat).Set(level), Line: lines[origin.Segment],
		}
	}
	for _, operand := range seen {
		for _, present := range operand {
			if err := budget.Step(); err != nil {
				return nil, false, err
			}
			if !present {
				return nil, false, nil
			}
		}
	}
	return mapped, true, nil
}

func fragmentGeometryKey(a, b Point,
	arithmetic ...*Arithmetic) string {
	c := ReplayArithmetic(arithmetic)
	u := c.Cmp(a.U, b.U)
	if !c.OK {
		return ""
	}
	if u > 0 || u == 0 && c.Cmp(a.V, b.V) > 0 {
		a, b = b, a
	}
	if !c.OK || !c.Valid(a.U) || !c.Valid(a.V) || !c.Valid(b.U) || !c.Valid(b.V) {
		return ""
	}
	return a.U.RatString() + "," + a.V.RatString() + ":" + b.U.RatString() + "," + b.V.RatString()
}
