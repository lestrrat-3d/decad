package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// This file is RecordChain's own admission-gate coverage
// (docs/sketch-seam-design.md §2.2, docs/surface-design.md §13.3): the same
// four gates RecordProfile earns, run over a chain instead of a profile.
// extrude_chain_test.go covers the stale, invalid, and uncertified-fragment
// gates end to end through ExtrudeChain; this file targets RecordChain
// directly, mirroring record_test.go's TestRecordProfileGates and its
// foreign/changed-snapshot siblings.

func TestRecordChainGates(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(40, 0)
	s.Fix(a)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]

	// Foreign: the chain's plane-local coordinates belong to its own sketch.
	other, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	_, _, err = decad.RecordChain(other, ch)
	require.ErrorIs(t, err, decad.ErrForeignProfile)

	// Nil input is degenerate, not a panic.
	_, _, err = decad.RecordChain(s, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, _, err = decad.RecordChain(nil, ch)
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// Stale: move the geometry after the snapshot.
	s.AddConstraint(sketch.NewDistance(a, b, 55))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.True(t, ch.IsStale(), "the solve should have moved the sketch under the chain")
	_, _, err = decad.RecordChain(s, ch)
	require.ErrorIs(t, err, decad.ErrStaleProfile)

	// A fresh chain records again.
	fresh := s.Chains()[0]
	_, _, err = decad.RecordChain(s, fresh)
	require.NoError(t, err)
}

// TestRecordChainRejectsForeignBoundaryEntity mirrors
// TestRecordProfileRejectsForeignBoundaryEntity: a chain edge naming an
// entity from a different sketch is ErrForeignProfile.
func TestRecordChainRejectsForeignBoundaryEntity(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	s.Fix(a)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]

	foreign, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	foreignA := foreign.CreatePoint(100, 100)
	foreign.Fix(foreignA)
	foreign.CreateLine(foreignA, foreign.CreatePoint(110, 100))
	_, err = foreign.Solve(t.Context())
	require.NoError(t, err)

	ch.Edges[0].Entity = foreign.Chains()[0].Entities[0]
	_, _, err = decad.RecordChain(s, ch)
	require.ErrorIs(t, err, decad.ErrForeignProfile)
}

// TestRecordChainRejectsTypedNilBoundaryEntity is RecordProfile's typed-nil
// case, over a chain: docs/surface-design.md §13.3's table groups a nil
// chain entity with the foreign-source row rather than an invalid one, so
// the sentinel here differs from RecordProfile's ErrInvalidProfile.
func TestRecordChainRejectsTypedNilBoundaryEntity(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(10, 0)
	s.Fix(a)
	s.CreateLine(a, b)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	ch := s.Chains()[0]
	ch.Edges[0].Entity = (*sketch.Line)(nil)

	_, _, err = decad.RecordChain(s, ch)
	require.ErrorIs(t, err, decad.ErrForeignProfile)
}

// TestRecordChainRejectsChangedSnapshot mirrors
// TestRecordProfileRejectsChangedCurrentBoundary: a caller-altered Edges
// slice matches no fresh Sketch.Chains() member.
func TestRecordChainRejectsChangedSnapshot(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateLine(s.CreatePoint(0, 0), s.CreatePoint(10, 0))
	s.CreateLine(s.CreatePoint(20, 0), s.CreatePoint(30, 0))
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	chains := s.Chains()
	require.Len(t, chains, 2)
	chains[0].Edges = chains[1].Edges

	_, _, err = decad.RecordChain(s, chains[0])
	require.ErrorIs(t, err, decad.ErrInvalidProfile)
}
