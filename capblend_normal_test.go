package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestCapBlendPlanePatchNormalOutwardBothCaps checks the sign of the Z
// component of every straight-wall chamfer patch's outward normal on the
// rectangular box, for BOTH the start and the end cap: since a start-cap
// band and an end-cap band tilt toward OPPOSITE caps (the exterior end of
// each band), the two cases must read opposite Z signs — the asymmetry a
// naive "the Plane's own u x v is always outward" assumption misses.
func TestCapBlendPlanePatchNormalOutwardBothCaps(t *testing.T) {
	for _, tc := range []struct {
		name    string
		end     bool
		wantPos bool // true: normal Z component must be positive
	}{
		{"end cap tilts toward +Z (the chamfered end)", true, true},
		{"start cap tilts toward -Z (the chamfered end)", false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, box := capBlendBox(t)
			chamfered, err := box.Chamfer(capLoopEdgesOn(box, tc.end), units.Millimeters(5))
			require.NoError(t, err)
			prefix := "chamferCap(end,"
			if !tc.end {
				prefix = "chamferCap(start,"
			}
			normals := patchNormalsByRolePrefix(t, chamfered, prefix)
			require.Len(t, normals, 4, "one Plane patch per rectangle wall")
			for role, n := range normals {
				if tc.wantPos {
					require.Greater(t, n.Z, 0.0, "role %s", role)
				} else {
					require.Less(t, n.Z, 0.0, "role %s", role)
				}
				require.InDelta(t, 1.0, n.Len(), 1e-9, "role %s: NormalAt must be unit", role)
			}
		})
	}
}

// TestCapBlendConePatchNormalOutwardBothCaps is the circular-rim analog of
// the Plane check: the cone chamfer patch's normal must tilt toward whichever
// cap is chamfered, for both caps.
func TestCapBlendConePatchNormalOutwardBothCaps(t *testing.T) {
	for _, tc := range []struct {
		name    string
		end     bool
		wantPos bool
	}{
		{"end cap", true, true},
		{"start cap", false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			const R, H = 30.0, 20.0
			disk := circleProfile(t, R, H)
			chamfered, err := disk.Chamfer(capLoopEdgesOn(disk, tc.end), units.Millimeters(5))
			require.NoError(t, err)
			prefix := "chamferCap(end,"
			if !tc.end {
				prefix = "chamferCap(start,"
			}
			normals := patchNormalsByRolePrefix(t, chamfered, prefix)
			require.Len(t, normals, 1, "one whole-turn Cone patch")
			for role, n := range normals {
				if tc.wantPos {
					require.Greater(t, n.Z, 0.0, "role %s", role)
				} else {
					require.Less(t, n.Z, 0.0, "role %s", role)
				}
				require.InDelta(t, 1.0, n.Len(), 1e-9, "role %s: NormalAt must be unit", role)
			}
		})
	}
}

// TestCapBlendReflexApexNormalOutward checks the reflex corner's Cone-apex
// patch against a reference derived from the topology alone, never from the
// builder's own formula.
//
// The offset is an EROSION, and at a reflex corner the eroded boundary is the
// arc of radius d about the corner with the surviving material radially
// OUTSIDE it — the sector the arc cuts off is exactly what the chamfer
// removes. So this patch's cone has the VOID inside and the solid outside,
// and its outward normal points radially INWARD, at the corner's own axis,
// while tilting toward the chamfered cap like every other patch in the band.
//
// The reference is built from the apex vertex, the arc foot and the cap face's
// own outward normal: with dc = ds = d the cone stands at 45 degrees, so the
// outward normal is (capNormal - radialUnit)/sqrt(2) exactly. The check is a
// vector identity rather than a pair of sign predicates, so an inverted normal
// fails it — and so does the radially-outward reading, which is EXACTLY
// perpendicular to the truth here and passes every sign test written against a
// cone ruling.
func TestCapBlendReflexApexNormalOutward(t *testing.T) {
	for _, tc := range []struct {
		name string
		end  bool
	}{
		{"end cap", true},
		{"start cap", false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			const d = 3.0
			body := reflexLBody(t)
			chamfered, err := body.Chamfer(capLoopEdgesOn(body, tc.end), units.Millimeters(d))
			require.NoError(t, err)

			prefix := "chamferCap(end,"
			capRole := roleCapEnd
			if !tc.end {
				prefix = "chamferCap(start,"
				capRole = roleCapStart
			}
			apex := apexPatchOf(t, chamfered, prefix)

			// The cap face's own outward normal is the band's axial reference;
			// it is independently pinned by the plane- and cone-patch tests.
			capFace := faceWithRole(t, chamfered, capRole)
			capN, err := capFace.NormalAt(capFace.Loops()[0].CoEdges()[0].Start().Position().Value)
			require.NoError(t, err)
			axis := capN.Value

			// The apex vertex is the ORIGINAL corner, held where the receiver
			// had it; the arc's own start is one foot of the offset connector.
			coedges := apex.Loops()[0].CoEdges()
			corner := coedges[1].End().Position().Value
			p := coedges[0].Start().Position().Value
			v := p.Sub(corner)
			require.InDelta(t, d, v.Dot(axis), 1e-9, "one setback along the cap's own outward sense")
			radial, ok := v.Sub(axis.Scale(v.Dot(axis))).Normalize()
			require.True(t, ok)

			want, ok := axis.Sub(radial).Normalize()
			require.True(t, ok)
			n, err := apex.NormalAt(p)
			require.NoError(t, err)
			require.InDelta(t, 1.0, n.Value.Dot(want), 1e-9, "the apex patch's outward normal")
			// Restated as the two facts the identity carries, so a failure
			// names which half moved.
			require.Less(t, n.Value.Dot(radial), 0.0, "toward the corner's own axis")
			require.Greater(t, n.Value.Dot(axis), 0.0, "toward the chamfered end")
		})
	}
}
