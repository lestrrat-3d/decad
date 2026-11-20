package booleaneval

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

func sourceGroup(owner *topo.Body, operand, ordinal int, f *topo.Face, flip bool,
	placed placedDepartures, walls map[*topo.Face]struct{}) facetedrecord.Group {
	group, _ := facetedrecord.SourceTag(f, flip)
	group = placed.charge(group, f, ordinal)
	group = chordedLoftPlane(group, owner, f)
	group = displacedWall(group, f, walls)
	// A mesh result face descends from f, so it carries every role f is
	// selected by: its own as Origins, which recognizers match faces by,
	// and those an analytic Boolean attributed to it as Upstream.
	group.Origins = slices.Clone(f.Origins)
	group.Upstream = slices.Clone(f.Upstream)
	// Identity only: no reader derives a plane or a bound from Source.
	group.Source = facetedrecord.FaceSource(owner.Origin.Producer, operand, ordinal, f)
	group.Planar = f.Surface.Kind() == surfacegeom.KindPlane || f.HeldPlanar
	return group
}

func configureChainGate(bm *meshbool.BoolMesh, pairLimit, allowance float64) {
	bm.PairChordLimit = pairLimit
	bm.ChainAllowance = allowance
	bm.Gate = max(pairLimit, allowance)
}

func summarizeChainAllowance(a *meshbool.BoolMesh, operandA int, b *meshbool.BoolMesh, operandB int) *ChainAllowanceSummary {
	var chosen *ChainAllowanceSummary
	for _, candidate := range []struct {
		mesh    *meshbool.BoolMesh
		operand int
	}{{a, operandA}, {b, operandB}} {
		m := candidate.mesh
		if !m.ChainAllowanceUsed {
			continue
		}
		summary := &ChainAllowanceSummary{Operand: candidate.operand,
			TouchedBound: m.ChainTouchedBound, PairChordLimit: m.PairChordLimit, ChainAllowance: m.ChainAllowance}
		if chosen == nil || summary.TouchedBound > chosen.TouchedBound {
			chosen = summary
		}
	}
	return chosen
}

// PrepBoolMesh lifts a tessellation into the exact domain and attaches its
// per-vertex and per-facet bounds. A non-finite vertex has no exact form, so
// it is rejected outright.
//
// A COLLAPSED operand facet — one whose exact normal is zero, which a rigid
// placement's own rounding can produce on an already-faceted body — is refused
// (decaderr.ErrUnsupported). A collapsed facet has no plane and no interior, so every
// contact predicate in this file is blind to it: a point or tangent contact the
// other operand makes THERE would be classified by nothing at all, and the
// facet would ride silently through the boolean with its component's verdict.
// The contact is exactly what this pipeline must not miss, so the operand is
// refused rather than partly examined. Loud beats silently wrong.
func PrepBoolMesh(ctx context.Context, m *topo.Mesh, src []int) (*meshbool.BoolMesh, error) {
	return prepBoolMesh(ctx, m, src, !retainedPlaneSlabDisabled(ctx))
}

// prepBoolMesh is PrepBoolMesh with the retained-surface reading as a
// parameter: a moved operand's facets lie on no retained plane or cylinder.
func prepBoolMesh(ctx context.Context, m *topo.Mesh, src []int, retained bool) (*meshbool.BoolMesh, error) {
	bm, err := meshbool.PrepContext(ctx, m.Vertices, m.Triangles, src)
	if err != nil {
		return nil, err
	}
	// Per-vertex bounds and their facet maxima (agent-docs/faceted-vertex-bounds-
	// design.md §2, §4.5): the mesh's own record, or §2.1's reading derived
	// from its face bounds.
	beta, err := topobuild.VertexBounds(m)
	if err != nil {
		return nil, err
	}
	bm.SetVertexBounds(beta)
	// A mesh whose planar facets carry a bound apart from their face's trim
	// chording (topo.Mesh.Planar) lowers each facet clear of every chord to
	// it, and hands the gate the rest for contacts that stay clear.
	if rec := m.Planar(); rec != nil {
		setPlanarBounds(bm, m, rec)
	}
	if retained {
		bm.SetChargedSlabBounds(retainedPlaneOf(m))
		bm.SetCylinderSlabBounds(retainedCylinderOf(m))
	}
	return bm, nil
}

// booleanProofBounds composes the independent, non-negative volume and area
// proof terms that survive the mesh boolean into its faceted result.
func booleanProofBounds(symA, symB, roundVol, slackA, slackB, dropArea float64) (
	volSymDiff, areaSlack float64,
) {
	return proofbound.AbsSumUpper(symA, symB, roundVol),
		proofbound.AbsSumUpper(slackA, slackB, dropArea)
}

// operandChord is the chord request for operand b of a pair whose tolerance,
// already scaled by chordScale, is pairTol (meshbool.OperandChord).
func operandChord(ctx context.Context, b *topo.Body, pairTol, dPair, chordScale float64) float64 {
	_, restating := booloperand.HeldFloor(b)
	return meshbool.OperandChord(pairTol, dPair*chordScale, booloperand.ChordShare(ctx, b).Floor*chordScale, restating)
}

// chordDiameter is the diameter a pair's freshly chorded operands are
// requested by: the overlap's when ctx allows it (WithOverlapChord) and
// a smaller diameter applies, else the pair's own.
func chordDiameter(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body, dPair float64) float64 {
	if !overlapAllowed(ctx) {
		return dPair
	}
	if d, choice := overlapDiameter(ctx, op, a, b, dPair); choice != PairDiameter {
		return d
	}
	return dPair
}

// CutChordChoice selects the diameter for freshly tessellated Cut operands.
type CutChordChoice int

const (
	PairDiameter CutChordChoice = iota
	OverlapDiameter
	TargetDiameter
)

// PreferredCutChord chooses a finer request when only a small region overlaps
// or a cylindrical tool extends outside a held target.
func PreferredCutChord(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body) CutChordChoice {
	_, dPair, err := booloperand.PairChord(ctx, a, b)
	if err != nil {
		return PairDiameter
	}
	_, choice := overlapDiameter(ctx, op, a, b, dPair)
	return choice
}

func overlapDiameter(ctx context.Context, op meshbool.OperationKind, a, b *topo.Body, dPair float64) (float64, CutChordChoice) {
	if op != meshbool.OpCut || !hasCurvedFace(b) {
		return 0, PairDiameter
	}
	target := booloperand.ChordShare(ctx, a)
	d, ok := meshbool.OverlapDiameter(target, booloperand.ChordShare(ctx, b))
	if !ok {
		return 0, PairDiameter
	}
	if c, clipped := clipOverlapDiameter(a, b); clipped && c < d {
		d = c
	}
	if d*meshbool.OverlapChordRatio <= dPair {
		return d, OverlapDiameter
	}
	// An extended cutter can enlarge the pair box even though its new bore wall
	// lies inside the target. Preserve that wall at the target's own chord.
	if _, held := a.Payload.(facetedrecord.Payload); !held {
		return 0, PairDiameter
	}
	p, ok := b.Payload.(prismrecord.Payload)
	if !ok || p.SurfaceResult || len(p.Profile.Holes) != 0 || len(p.Profile.Outer.Segments) != 1 {
		return 0, PairDiameter
	}
	c, ok := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !c.CCW || c.TStart != 0 || c.TEnd != 1 {
		return 0, PairDiameter
	}
	d = target.Hi.Sub(target.Lo).Len()
	if d > 0 && d < dPair {
		return d, TargetDiameter
	}
	return 0, PairDiameter
}

type overlapChordKey struct{}

// WithOverlapChord marks ctx so a Cut with a smaller preferred diameter chords its
// fresh operands by the overlap. Only the caller that retries the Cut at the
// pair's own chord on any refusal sets it (booleandispatch.BodyScaled).
func WithOverlapChord(ctx context.Context) context.Context {
	return context.WithValue(ctx, overlapChordKey{}, true)
}

// WithoutOverlapChord clears WithOverlapChord for the Booleans ctx reaches.
func WithoutOverlapChord(ctx context.Context) context.Context {
	return context.WithValue(ctx, overlapChordKey{}, false)
}

func overlapAllowed(ctx context.Context) bool {
	v, _ := ctx.Value(overlapChordKey{}).(bool)
	return v
}

// hasCurvedFace reports whether some face of b is not a plane.
func hasCurvedFace(b *topo.Body) bool {
	for _, l := range b.Lumps {
		for _, sh := range l.Shells {
			for _, f := range sh.Faces {
				if _, ok := f.Surface.(surfacegeom.Plane); !ok {
					return true
				}
			}
		}
	}
	return false
}

// setPlanarBounds attaches rec to bm and lowers every facet whose own box is
// clear of its face's chords to its planar bound.
func setPlanarBounds(bm *meshbool.BoolMesh, m *topo.Mesh, rec *topo.PlanarRecord) {
	n := len(m.Triangles)
	bm.PlanarOwn = rec.Own
	bm.PlanarChords = make([][][2]r3.Vec, n)
	bm.PlanarReach = make([]float64, n)
	for t := range n {
		f := m.Source[t]
		bm.PlanarChords[t] = rec.Chords[f]
		bm.PlanarReach[t] = rec.Reach[f]
		if rec.Own[t] < bm.FacetBound[t] && meshbool.ChordClear(bm.Boxes[t][0], bm.Boxes[t][1], rec.Chords[f], rec.Reach[f]) {
			bm.FacetBound[t] = rec.Own[t]
			bm.SlabBound[t] = min(bm.SlabBound[t], rec.Own[t])
		}
	}
}
