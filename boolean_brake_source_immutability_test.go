package decad

import (
	"bytes"
	"encoding/json"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type brakeSourceFaceRecord struct {
	face                        *Face
	cap                         *topo.CapRecord
	denoted                     *surfacenormal.Revolved
	capZ                        uint64
	capLevel                    string
	capOrigin, capU, capV, capN r3.Vec
	capTransform                r3.Transform
	denotedJSON                 []byte
}

func TestBrakeSourceFaceRecordsSurvivePlacement(t *testing.T) {
	box := brakeFixtureBox(t, t.Context(), New(), -1, 1, -1, 1, -1, 1)
	round := departureRevolve(t, [][2]float64{{0, 2}, {3, 2}, {3, 4}, {0, 4}}, 0)
	for _, tc := range []struct {
		name                 string
		body                 *Body
		wantCap, wantDenoted bool
	}{{"cap", box, true, false}, {"denoted", round, false, true}} {
		t.Run(tc.name, func(t *testing.T) {
			var snapshots []brakeSourceFaceRecord
			foundCap, foundDenoted := false, false
			for _, face := range tc.body.Faces() {
				record := brakeSourceFaceRecord{face: face, cap: face.n.CapRecord, denoted: face.n.Denoted}
				if record.cap != nil {
					foundCap = true
					record.capZ = math.Float64bits(record.cap.Z)
					if record.cap.ZDenoted != nil {
						record.capLevel = record.cap.ZDenoted.RatString()
					}
					record.capOrigin, record.capU, record.capV, record.capN = record.cap.Frame.Origin(), record.cap.Frame.U(), record.cap.Frame.V(), record.cap.Frame.N()
					record.capTransform = record.cap.Xform
				}
				if record.denoted != nil {
					foundDenoted = true
					var err error
					record.denotedJSON, err = json.Marshal(record.denoted) //nolint:musttag // This checks internal record stability, not a wire format.
					if err != nil {
						t.Fatal(err)
					}
				}
				snapshots = append(snapshots, record)
			}
			if tc.wantCap && !foundCap || tc.wantDenoted && !foundDenoted {
				t.Fatal("source record missing")
			}
			move, err := r3.Translation(r3.Vec{X: 1})
			if err != nil {
				t.Fatal(err)
			}
			if _, err := Transform(t.Context(), tc.body, move); err != nil {
				t.Fatal(err)
			}
			for i, record := range snapshots {
				face := record.face
				if face.n.CapRecord != record.cap || face.n.Denoted != record.denoted {
					t.Fatalf("face %d source pointer changed", i)
				}
				if record.cap != nil {
					c := record.cap
					level := ""
					if c.ZDenoted != nil {
						level = c.ZDenoted.RatString()
					}
					if math.Float64bits(c.Z) != record.capZ || level != record.capLevel || c.Frame.Origin() != record.capOrigin || c.Frame.U() != record.capU || c.Frame.V() != record.capV || c.Frame.N() != record.capN || c.Xform != record.capTransform {
						t.Fatalf("face %d cap changed", i)
					}
				}
				if record.denoted != nil {
					now, err := json.Marshal(record.denoted) //nolint:musttag // This checks internal record stability, not a wire format.
					if err != nil || !bytes.Equal(now, record.denotedJSON) {
						t.Fatalf("face %d denotation changed: %v", i, err)
					}
				}
			}
		})
	}
}
