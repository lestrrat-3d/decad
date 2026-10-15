package decad

import (
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesToolCapArcBindsLiveSourceCells(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	sides, ok := revolveLoftFacesToolCapSidesOf(proof)
	require.True(t, ok)
	box, err := lfSourceBox(proof.trim.Source)
	require.NoError(t, err)
	checked := 0
	for _, circle := range sides.circles {
		if circle.tool != 1 || circle.sourceMiss {
			continue
		}
		span := proof.section.Spans[circle.span]
		var arc *revolveLoftFacesToolCapOccupiedArc
		var certified bool
		if span.Start.U == span.End.U {
			arc, certified, err = revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, circle)
		} else {
			arc, certified, err = revolveLoftFacesHeelSlopedOccupiedArc(t.Context(), proof, circle)
		}
		require.NoError(t, err)
		require.True(t, certified)
		band, time, unique := loftface.ConstantXBand(proof.trim.Maps, box, circle.x)
		require.True(t, unique)
		require.Equal(t, band, arc.band)
		require.Zero(t, time.Cmp(arc.time))
		current, currentErr := revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, arc)
		require.NoError(t, currentErr)
		require.True(t, current)
		if span.Start.U != span.End.U {
			lower := lfZero()
			if circle.x.Cmp(proof.trim.Toe.apex) < 0 {
				lower = lfDiv(lfSub(proof.trim.Toe.apex, circle.x), proof.trim.Toe.slope)
			}
			section, sourceOK, sourceErr := revolveLoftFacesConstantXSourceCells(t.Context(), proof,
				circle.x, lower, circle.radius)
			require.NoError(t, sourceErr)
			require.True(t, sourceOK)
			changedSection := *section
			changedSection.cells = slices.Clone(section.cells)
			changedArcCell := false
			for i := range changedSection.cells {
				if changedSection.cells[i].source != nil {
					continue
				}
				changedSection.cells[i].source = make(freeform.BezierSpan, 0)
				changedArcCell = true
				break
			}
			require.True(t, changedArcCell)
			_, certified, sourceErr = revolveLoftFacesHeelOccupiedFromSource(t.Context(), proof,
				circle, &changedSection)
			require.NoError(t, sourceErr)
			require.False(t, certified, "an empty spline record cannot replace the live whole Arc")
		}

		changed := *arc
		changed.band++
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another cap band cannot own the same arc")
		changed = *arc
		changed.time = new(big.Rat).Add(arc.time, big.NewRat(1, 1<<20))
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another time cannot own the same source cells")
		changed = *arc
		changed.negativeZ.parameter.Lo = new(big.Rat).Add(
			arc.negativeZ.parameter.Lo, arc.negativeZ.parameter.Hi)
		changed.negativeZ.parameter.Lo.Quo(changed.negativeZ.parameter.Lo, big.NewRat(2, 1))
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "a narrowed root bracket is not the certified source bracket")
		changed = *arc
		changed.positiveZ.face = bornFace(&Face{})
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another retained face cannot own the source crossing")
		checked++
	}
	require.Equal(t, 2, checked, "the real source has vertical and sloped heel arcs")
}

func TestRevolveLoftFacesToolCapSourceCellsPreserveSlicePresence(t *testing.T) {
	original := &revolveLoftFacesConstantXSource{}
	changed := *original
	changed.cells = []revolveLoftFacesVerticalCell{}
	require.False(t, revolveLoftFacesToolCapSourceCellsEqual(original, &changed))

	original.cells = []revolveLoftFacesVerticalCell{{}}
	changed = *original
	changed.cells = slices.Clone(original.cells)
	changed.cells[0].source = make(freeform.BezierSpan, 0)
	require.False(t, revolveLoftFacesToolCapSourceCellsEqual(original, &changed))

	changed.cells[0] = original.cells[0]
	changed.cells[0].retained = []loftface.VerticalPiece{}
	require.False(t, revolveLoftFacesToolCapSourceCellsEqual(original, &changed))
}
