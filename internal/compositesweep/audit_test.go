package compositesweep_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// planeExtent reads every span as touching the z = 0 plane from its own
// side, so each adjacent pair passes the opposite-sides rule exactly.
type planeExtent struct{}

func (planeExtent) AuditExtent(context.Context, r3.Vec, *freeform.FreeformWork) (float64, float64, float64, error) {
	return 0, 0, 0, nil
}

type fixedSections struct {
	start, end compositesweep.AuditBox
}

func (s fixedSections) AuditSectionBoxes(
	context.Context, *freeform.FreeformWork,
) (compositesweep.AuditBox, compositesweep.AuditBox, error) {
	return s.start, s.end, nil
}

func sectionBox(x0, x1 float64) compositesweep.AuditBox {
	return compositesweep.AuditBox{Min: r3.NewVec(x0, 0, 0), Max: r3.NewVec(x1, 0.5, 0)}
}

func auditTestSpan(t *testing.T, coplanar bool, start, end compositesweep.AuditBox) compositesweep.AuditSpan {
	t.Helper()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return compositesweep.AuditSpan{
		StartPlane: surfacegeom.Plane{Frame: frame}, StartPlanar: true, EndPlanar: true,
		CoplanarSections: coplanar,
		Extent:           planeExtent{},
		Sections:         fixedSections{start: start, end: end},
	}
}

const farSectionRefusal = "both keep their far sections in their shared plane"

func TestAuditCoplanarSectionGap(t *testing.T) {
	t.Parallel()

	t.Run("overlapping far sections refuse", func(t *testing.T) {
		t.Parallel()
		err := compositesweep.Audit(t.Context(), []compositesweep.AuditSpan{
			auditTestSpan(t, true, sectionBox(0, 2), sectionBox(8, 10)),
			auditTestSpan(t, true, sectionBox(8, 10), sectionBox(1, 3)),
		})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "sweep spans 0 and 1 "+farSectionRefusal)
	})
	t.Run("separated far sections admit", func(t *testing.T) {
		t.Parallel()
		require.NoError(t, compositesweep.Audit(t.Context(), []compositesweep.AuditSpan{
			auditTestSpan(t, true, sectionBox(0, 2), sectionBox(8, 10)),
			auditTestSpan(t, true, sectionBox(8, 10), sectionBox(3, 5)),
		}))
	})
	// A span whose sections are not coplanar meets the shared plane only in
	// the shared section, so its neighbour's far section has nothing to meet.
	for _, tc := range []struct {
		name          string
		before, after bool
	}{
		{name: "only before coplanar admits", before: true},
		{name: "only after coplanar admits", after: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.NoError(t, compositesweep.Audit(t.Context(), []compositesweep.AuditSpan{
				auditTestSpan(t, tc.before, sectionBox(0, 2), sectionBox(8, 10)),
				auditTestSpan(t, tc.after, sectionBox(8, 10), sectionBox(1, 3)),
			}))
		})
	}
	t.Run("missing section reader refuses", func(t *testing.T) {
		t.Parallel()
		after := auditTestSpan(t, true, sectionBox(8, 10), sectionBox(3, 5))
		after.Sections = nil
		err := compositesweep.Audit(t.Context(), []compositesweep.AuditSpan{
			auditTestSpan(t, true, sectionBox(0, 2), sectionBox(8, 10)), after,
		})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "sweep spans 0 and 1 have coplanar sections but no section boxes")
	})
}

func TestAuditClosedCoplanarSectionGap(t *testing.T) {
	t.Parallel()

	// The closing pair runs from span 2 to span 0: span 2's start section
	// faces span 0's end section. Span 1 is not coplanar, so the two open
	// joins never compare far sections.
	t.Run("closing pair far sections refuse", func(t *testing.T) {
		t.Parallel()
		err := compositesweep.AuditClosed(t.Context(), []compositesweep.AuditSpan{
			auditTestSpan(t, true, sectionBox(20, 22), sectionBox(1, 3)),
			auditTestSpan(t, false, sectionBox(1, 3), sectionBox(5, 7)),
			auditTestSpan(t, true, sectionBox(0, 2), sectionBox(20, 22)),
		})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "sweep spans 2 and 0 "+farSectionRefusal)
	})
	t.Run("closing pair reads far sections in path order", func(t *testing.T) {
		t.Parallel()
		// Span 2's end and span 0's start overlap, but they are the closing
		// section itself, not far sections.
		require.NoError(t, compositesweep.AuditClosed(t.Context(), []compositesweep.AuditSpan{
			auditTestSpan(t, true, sectionBox(20, 22), sectionBox(5, 7)),
			auditTestSpan(t, false, sectionBox(5, 7), sectionBox(9, 11)),
			auditTestSpan(t, true, sectionBox(0, 2), sectionBox(20, 22)),
		}))
	})
}
