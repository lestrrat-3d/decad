package parts

import (
	"fmt"
	"math"
	"slices"
	"strings"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/explodeplan"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Clearing says what a stage's members must clear.
type Clearing int

const (
	ClearJoined Clearing = iota // clear the parent and the joined parts already moved
	ClearAll                    // clear every part that stays put, by the gap
)

// String returns the lower-case words for the clearing.
func (c Clearing) String() string {
	if c == ClearAll {
		return "all"
	}
	return "joined"
}

// PathState says what a part's straight path met, judged by boxes alone.
type PathState int

const (
	PathNone     PathState = iota // the part did not move
	PathClear                     // its swept box meets no part it is not joined to
	PathBlocked                   // it starts clear and its sweep meets a part
	PathEnclosed                  // its start box already meets a part's box, so a box test cannot decide
	PathStuck                     // no distance clears it
)

// String returns the lower-case word for the state.
func (p PathState) String() string {
	switch p {
	case PathClear:
		return "clear"
	case PathBlocked:
		return "blocked"
	case PathEnclosed:
		return "enclosed"
	case PathStuck:
		return "stuck"
	}
	return "none"
}

// Stage is one step of a staged explosion. Its parts move in it, in the order of
// the joint walk; every other part stays where earlier stages left it.
type Stage struct {
	Name   string   // non-empty and unique
	Parts  []string // part names
	Groups []string // container paths (Set.Group); every part in one, or below it, is a member
	// Gap is how far past clearing each member moves; the zero Value means
	// WithExplodeGap's gap.
	Gap units.Value
	// Clear says what the members must clear: ClearJoined (the zero value) or
	// ClearAll.
	Clear Clearing
	// Direction, when not zero, moves every member along it instead of along its
	// joint (Pull then reads StageDirection). Its length does not matter.
	Direction r3.Vec
	// Together moves the members as one rigid group by one offset.
	Together bool
}

// WithExplodeStages runs the explosion in the given stages, in order. A part no
// stage names, other than the ground, forms a last stage named "" that clears its
// joined neighbours by WithExplodeGap's gap. With no stages every part is in that
// last stage.
func WithExplodeStages(stages ...Stage) ExplodeOption {
	return option{func(c *config) error {
		c.stages = append(c.stages, stages...)
		return nil
	}}
}

// WithPartGap sets one part's gap, in place of its stage's and
// WithExplodeGap's. A part no Set has is decad.ErrDegenerate at the query.
func WithPartGap(part string, gap units.Value) ExplodeOption {
	if err := checkLength(gap, "part gap", false); err != nil {
		return failing(err)
	}
	return option{func(c *config) error {
		if c.partGaps == nil {
			c.partGaps = map[string]units.Value{}
		}
		c.partGaps[part] = gap
		return nil
	}}
}

// WithExplodeScale multiplies every final offset by scale, a positive
// Dimensionless value: a uniform spread about the assembled layout. The default
// is units.Scalar(1).
func WithExplodeScale(scale units.Value) ExplodeOption {
	switch b := scale.Base(); {
	case scale.Kind() != units.Dimensionless:
		return failing(fmt.Errorf("parts: explode scale: %w", decad.ErrUnitKind))
	case math.IsNaN(b) || math.IsInf(b, 0):
		return failing(fmt.Errorf("parts: explode scale: %w", decad.ErrNotFinite))
	case b < 0:
		return failing(fmt.Errorf("parts: explode scale: %w", decad.ErrNegativeMagnitude))
	case b == 0:
		return failing(fmt.Errorf("parts: %w: explode scale must be positive", decad.ErrDegenerate))
	default:
		return option{func(c *config) error { c.scale = b; return nil }}
	}
}

// resolvedStages is the stages checked against the Set, in plan indices.
type resolvedStages struct {
	names   []string // the stage names, "" for the implicit last stage
	stages  []explodeplan.Stage
	partGap []float64 // per plan index, NaN when unset
}

// resolveStages checks the stages and part gaps against the Set (S10) and turns
// them into plan stages. plan maps part indices to plan indices.
func (s *Set) resolveStages(c config, ground int, idx []int, plan map[int]int) (resolvedStages, error) {
	var out resolvedStages
	out.partGap = make([]float64, len(idx))
	for i := range out.partGap {
		out.partGap[i] = math.NaN()
	}
	for name, g := range c.partGaps {
		i, ok := s.index[name]
		k, bodied := plan[i]
		if !ok || !bodied {
			return out, fmt.Errorf("parts: %w: part gap names %q, which is unknown or has no body", decad.ErrDegenerate, name)
		}
		out.partGap[k] = g.Base()
	}
	named := map[int]struct{}{}
	seen := map[string]struct{}{}
	for _, st := range c.stages {
		if st.Name == "" {
			return out, fmt.Errorf("parts: %w: a stage needs a name", decad.ErrDegenerate)
		}
		if _, dup := seen[st.Name]; dup {
			return out, fmt.Errorf("parts: %w: stage %q is repeated", decad.ErrDegenerate, st.Name)
		}
		seen[st.Name] = struct{}{}
		if !finiteVec(st.Direction) {
			return out, fmt.Errorf("parts: stage %q direction: %w", st.Name, decad.ErrNotFinite)
		}
		if err := checkLength(st.Gap, "stage "+st.Name+" gap", false); err != nil {
			return out, err
		}
		members, err := s.stageMembers(st, plan)
		if err != nil {
			return out, err
		}
		if slices.Contains(members, ground) && st.Direction == (r3.Vec{}) {
			return out, fmt.Errorf("parts: %w: stage %q moves the ground part and states no direction", decad.ErrDegenerate, st.Name)
		}
		for _, m := range members {
			named[m] = struct{}{}
		}
		gap := math.NaN()
		if st.Gap != (units.Value{}) {
			gap = st.Gap.Base()
		}
		out.names = append(out.names, st.Name)
		out.stages = append(out.stages, explodeplan.Stage{Members: members, Gap: gap, Clear: explodeplan.Clearing(st.Clear),
			Dir: st.Direction, Together: st.Together})
	}
	var rest []int
	for k := range idx {
		if _, ok := named[k]; !ok && k != ground {
			rest = append(rest, k)
		}
	}
	if len(rest) > 0 {
		out.names = append(out.names, "")
		out.stages = append(out.stages, explodeplan.Stage{Members: rest, Gap: math.NaN()})
	}
	return out, nil
}

// stageMembers resolves a stage's parts and groups into plan indices.
func (s *Set) stageMembers(st Stage, plan map[int]int) ([]int, error) {
	var members []int
	add := func(k int, what string) error {
		if slices.Contains(members, k) {
			return fmt.Errorf("parts: %w: stage %q names %s twice", decad.ErrDegenerate, st.Name, what)
		}
		members = append(members, k)
		return nil
	}
	for _, name := range st.Parts {
		i, ok := s.index[name]
		k, bodied := plan[i]
		if !ok || !bodied {
			return nil, fmt.Errorf("parts: %w: stage %q names part %q, which is unknown or has no body", decad.ErrDegenerate,
				st.Name, name)
		}
		if err := add(k, fmt.Sprintf("part %q", name)); err != nil {
			return nil, err
		}
	}
	for _, g := range st.Groups {
		found := false
		for i, p := range s.parts {
			if p.Body == nil {
				continue
			}
			path, err := s.Group(p.Name)
			if err != nil {
				return nil, err
			}
			if path != g && (g == "" || !strings.HasPrefix(path, g+"/")) {
				continue
			}
			found = true
			if err := add(plan[i], fmt.Sprintf("part %q through group %q", p.Name, g)); err != nil {
				return nil, err
			}
		}
		if !found {
			return nil, fmt.Errorf("parts: %w: stage %q names group %q, which holds no part with a body", decad.ErrDegenerate,
				st.Name, g)
		}
	}
	if len(members) == 0 {
		return nil, fmt.Errorf("parts: %w: stage %q has no members", decad.ErrDegenerate, st.Name)
	}
	return members, nil
}
