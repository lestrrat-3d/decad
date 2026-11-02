package stackedrecord_test

import (
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestBuildWithBothWallWitnessesPreservesSealedFaces(t *testing.T) {
	frame := xyFrame(t, 0)
	circle := func(x float64) momentinput.Profile {
		return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: x},
				Radius: units.Millimeters(1.5), CCW: true, TEnd: 1},
		}}}
	}
	for _, tc := range []struct {
		name      string
		profiles  []momentinput.Profile
		circleN   int
		rectangle int
	}{
		{"circle", []momentinput.Profile{circle(-5), circle(5)}, 2, 0},
		{"rectangle", []momentinput.Profile{rectRecord(-6, -1, -4, 1),
			rectRecord(4, -1, 6, 1)}, 0, 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			sp := stackedrecord.Payload{Slabs: []stackedrecord.Slab{{Regions: tc.profiles, Z1: 2}},
				Frame: frame, Xform: r3.Identity()}
			env := topo.Env{Doc: &topo.Document{}, Ref: 9, Alloc: topotest.Alloc{}}
			body, circles, rectangles, err := stackedrecord.BuildWithBothWallWitnesses(t.Context(), env, sp)
			if err != nil || body == nil || len(circles) != tc.circleN || len(rectangles) != tc.rectangle {
				t.Fatalf("body=%t circles=%d rectangles=%d error=%v",
					body != nil, len(circles), len(rectangles), err)
			}
			if !body.HasStrictPrismBuildSeal() {
				t.Fatal("compatibility build lost its strict source seal")
			}
			faces := topobuild.Faces(body)
			seen := func(face *topo.Face) bool {
				return slices.Contains(faces, face)
			}
			for _, w := range circles {
				if !seen(w.Face) || !seen(w.CapStart) || !seen(w.CapEnd) || w.CapAmbiguous {
					t.Fatal("circle witness lacks built wall or cap pointers")
				}
			}
			for _, w := range rectangles {
				if !seen(w.CapStart) || !seen(w.CapEnd) || w.CapAmbiguous {
					t.Fatal("rectangle witness lacks built cap pointers")
				}
				for _, face := range w.Walls {
					if !seen(face) {
						t.Fatal("rectangle witness lacks a built wall pointer")
					}
				}
			}
		})
	}
}
