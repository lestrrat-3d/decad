package decad

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const (
	loftOperandPrismFirst = "prism_first"
	loftOperandLoftFirst  = "loft_first"
)

func TestLoftContactDirectShellUnionVaryingRectangleBothOrders(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		name := loftOperandPrismFirst
		if reverse {
			name = loftOperandLoftFirst
		}
		t.Run(name, func(t *testing.T) {
			world := sketch.NewWorld()
			base := world.XY()
			upper, err := world.CreateOffsetPlane(base, 4)
			require.NoError(t, err)
			var sections [2]*sketch.Sketch
			var profiles [2]*sketch.Profile
			for i, coords := range [2][4]float64{
				{8, 4, 12, 6}, {8, 4, 13, 6},
			} {
				plane := base
				if i == 1 {
					plane = upper
				}
				s, createErr := world.CreateSketch(plane)
				require.NoError(t, createErr)
				rect := s.CreateRectangle(coords[0], coords[1], coords[2], coords[3])
				s.Fix(rect.A)
				_, createErr = s.Solve(t.Context())
				require.NoError(t, createErr)
				sections[i], profiles[i] = s, s.Profiles()[0]
			}
			d := New()
			loft, err := Loft(t.Context(), d, sections[0], profiles[0],
				sections[1], profiles[1])
			require.NoError(t, err)
			prismSketch, err := world.CreateSketch(base)
			require.NoError(t, err)
			rect := prismSketch.CreateRectangle(0, 0, 10, 10)
			prismSketch.Fix(rect.A)
			_, err = prismSketch.Solve(t.Context())
			require.NoError(t, err)
			prism, err := Extrude(t.Context(), d, prismSketch,
				prismSketch.Profiles()[0], Distance{D: units.Millimeters(5), Dir: Along})
			require.NoError(t, err)
			first, second := prism, loft
			if reverse {
				first, second = loft, prism
			}
			if !reverse {
				// A pair outside the prism/Loft source class still falls through.
				miss, attempted, missErr := tryLoftContactDirectShellUnion(t.Context(),
					meshbool.OpUnion, d, d.nextProducerID(), prism, prism)
				require.NoError(t, missErr)
				require.False(t, attempted)
				require.True(t, miss == nil)
			}
			candidate, admitted, err := tryLoftContactDirectShellUnion(t.Context(),
				meshbool.OpUnion, d, d.nextProducerID(), first, second)
			require.NoError(t, err)
			require.True(t, admitted, "the public case must reach the direct shell")
			require.True(t, candidate != nil)
			if !reverse {
				operation := proofbound.NewWorkBudget(t.Context())
				boundary, boundaryOK, boundaryErr := certifyLoftOutputBoundaryForDirectShell(
					t.Context(), prism, loft, operation)
				require.NoError(t, boundaryErr)
				require.True(t, boundaryOK)
				pp := prism.n.Payload.(prismPayload)
				lp := loft.n.Payload.(loftPayload)
				proof, proofOK, proofErr := loftVolumeCompose(t.Context(), operation,
					pp, lp, boundary)
				require.NoError(t, proofErr)
				require.True(t, proofOK)
				require.Less(t, proof.totalOutputVisits, uint64(loftVolumeOutputSteps))
				// A one-step remainder must refuse before conversion can build.
				refused, accepted, refusalErr := loftContactFacetedBody(t.Context(), d,
					d.nextProducerID(), first, second, prism, loft, lp, proof,
					proofbound.NewWallWorkBudgetWithOperation(1, operation))
				require.ErrorIs(t, refusalErr, ErrUnsupported)
				require.ErrorContains(t, refusalErr, "direct Loft/prism bounded proof limit")
				require.False(t, accepted)
				require.True(t, refused == nil)

				// If cancellation and wall exhaustion meet on the same step,
				// the operation-aware budget must report cancellation.
				raceCtx, stop := context.WithCancel(t.Context())
				defer stop()
				raceOperation := &proofbound.WorkBudget{
					StepFn: func() error {
						stop()
						return nil
					},
					ErrFn: raceCtx.Err,
				}
				raceBody, accepted, raceErr := loftContactFacetedBody(raceCtx, d,
					d.nextProducerID(), first, second, prism, loft, lp, proof,
					proofbound.NewWallWorkBudgetWithOperation(1, raceOperation))
				require.ErrorIs(t, raceErr, context.Canceled)
				require.False(t, accepted)
				require.True(t, raceBody == nil)

				// This cancellation occurs during the initial source-face scan.
				// It does not establish cancellation in audit or body build.
				cancelCtx, cancel := context.WithCancel(t.Context())
				defer cancel()
				steps := 0
				cancelBudget := &proofbound.WorkBudget{
					StepFn: func() error {
						steps++
						if steps == 25 {
							cancel()
						}
						return cancelCtx.Err()
					},
					ErrFn: cancelCtx.Err,
				}
				cancelled, accepted, cancelErr := loftContactFacetedBody(cancelCtx,
					d, d.nextProducerID(), first, second, prism, loft, lp, proof,
					cancelBudget)
				require.ErrorIs(t, cancelErr, context.Canceled)
				require.False(t, accepted)
				require.True(t, cancelled == nil)
				require.Equal(t, 25, steps)
			}
			before := append([]*Body(nil), d.Bodies()...)
			beforeProducer := d.nextProducerID()
			firstDoc, secondDoc := first.n.Doc, second.n.Doc
			firstOrigin, secondOrigin := first.n.Origin, second.n.Origin
			firstLumps, secondLumps := first.Lumps(), second.Lumps()
			firstKind, secondKind := first.n.Kind, second.n.Kind
			firstSolid, secondSolid := first.n.Solid, second.n.Solid
			firstVolume, secondVolume := first.n.Volume.Value.Base(), second.n.Volume.Value.Base()
			firstVolumeBound, secondVolumeBound := first.n.Volume.Bound.Base(), second.n.Volume.Bound.Base()
			firstVolumeExact, secondVolumeExact := first.n.Volume.Exactness, second.n.Volume.Exactness
			limited := context.WithValue(t.Context(), loftDirectOutputLimitKey{}, uint64(1))
			refused, refusalErr := Union(limited, first, second)
			require.True(t, refused == nil)
			require.ErrorIs(t, refusalErr, ErrUnsupported)
			require.ErrorContains(t, refusalErr, "direct Loft/prism bounded proof limit")
			after := d.Bodies()
			require.Equal(t, len(before), len(after), "the refusal must not commit a body")
			for i := range before {
				require.True(t, after[i] == before[i], "body %d changed after refusal", i)
			}
			require.True(t, first.n.Owner == first && second.n.Owner == second,
				"the refusal must preserve both node owners")
			require.True(t, first.n.Doc == firstDoc && second.n.Doc == secondDoc,
				"the refusal must preserve both owning documents")
			require.True(t, first.n.Origin == firstOrigin && second.n.Origin == secondOrigin,
				"the refusal must preserve both source identities")
			require.True(t, first.n.Kind == firstKind && second.n.Kind == secondKind,
				"the refusal must preserve both body kinds")
			require.True(t, first.n.Solid == firstSolid && second.n.Solid == secondSolid,
				"the refusal must preserve both solid flags")
			require.Equal(t, firstVolume, first.n.Volume.Value.Base(),
				"the refusal must preserve the first operand volume")
			require.Equal(t, secondVolume, second.n.Volume.Value.Base(),
				"the refusal must preserve the second operand volume")
			require.True(t, first.n.Volume.Bound.Base() == firstVolumeBound &&
				second.n.Volume.Bound.Base() == secondVolumeBound,
				"the refusal must preserve both volume bounds")
			require.True(t, first.n.Volume.Exactness == firstVolumeExact &&
				second.n.Volume.Exactness == secondVolumeExact,
				"the refusal must preserve both volume exactness values")
			afterFirstLumps, afterSecondLumps := first.Lumps(), second.Lumps()
			require.Equal(t, len(firstLumps), len(afterFirstLumps))
			require.Equal(t, len(secondLumps), len(afterSecondLumps))
			for i := range firstLumps {
				require.True(t, afterFirstLumps[i] == firstLumps[i],
					"first operand lump %d changed after refusal", i)
			}
			for i := range secondLumps {
				require.True(t, afterSecondLumps[i] == secondLumps[i],
					"second operand lump %d changed after refusal", i)
			}
			require.Equal(t, beforeProducer, d.nextProducerID(),
				"the refusal must not advance the document producer")

			result, err := Union(t.Context(), first, second)
			require.NoError(t, err)
			require.True(t, result.IsSolid())
			volume, err := result.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-520),
				volume.Bound.Base(), "measured=%g bound=%g",
				volume.Value.Base(), volume.Bound.Base())
			for _, role := range []string{"side(0,0,0)", "side(0,0,1)"} {
				ref := featureRefOf(provenance.Ref{
					Producer: loft.n.Origin.Producer, Role: role,
				})
				faces, selectErr := Faces(FaceCreatedBy(ref)).SelectFaces(result)
				require.NoError(t, selectErr)
				require.Greater(t, len(faces), 0,
					"missing public Loft provenance role %q", role)
			}
			for _, edge := range result.Edges() {
				require.Equal(t, 2, len(edge.Faces()))
			}
			report, err := d.Verify(t.Context())
			require.NoError(t, err)
			require.True(t, report.Passed(), "status=%v bodies=%d",
				report.Status, len(report.Bodies))
		})
	}
	t.Run("trapezoid_524", func(t *testing.T) {
		for _, reverse := range []bool{false, true} {
			name := loftOperandPrismFirst
			if reverse {
				name = loftOperandLoftFirst
			}
			t.Run(name, func(t *testing.T) {
				world := sketch.NewWorld()
				base := world.XY()
				upper, err := world.CreateOffsetPlane(base, 4)
				require.NoError(t, err)
				var sections [2]*sketch.Sketch
				var profiles [2]*sketch.Profile
				for cap, corners := range [2][4][2]float64{
					{{8, 4}, {12, 4}, {13, 6}, {8, 6}},
					{{8, 4}, {13, 4}, {14, 6}, {8, 6}},
				} {
					plane := base
					if cap == 1 {
						plane = upper
					}
					s, createErr := world.CreateSketch(plane)
					require.NoError(t, createErr)
					var points [4]*sketch.Point
					for i, xy := range corners {
						points[i] = s.CreatePoint(xy[0], xy[1])
						s.Fix(points[i])
					}
					for i := range points {
						s.CreateLine(points[i], points[(i+1)%len(points)])
					}
					_, createErr = s.Solve(t.Context())
					require.NoError(t, createErr)
					found := s.Profiles()
					require.Equal(t, 1, len(found))
					require.True(t, found[0].Valid)
					sections[cap], profiles[cap] = s, found[0]
				}
				d := New()
				prismSketch, err := world.CreateSketch(base)
				require.NoError(t, err)
				rect := prismSketch.CreateRectangle(0, 0, 10, 10)
				prismSketch.Fix(rect.A)
				_, err = prismSketch.Solve(t.Context())
				require.NoError(t, err)
				prismProfiles := prismSketch.Profiles()
				require.Equal(t, 1, len(prismProfiles))
				prism, err := Extrude(t.Context(), d, prismSketch, prismProfiles[0],
					Distance{D: units.Millimeters(5), Dir: Along})
				require.NoError(t, err)
				loft, err := Loft(t.Context(), d, sections[0], profiles[0], sections[1], profiles[1])
				require.NoError(t, err)
				first, second := prism, loft
				if reverse {
					first, second = loft, prism
				}
				candidate, admitted, err := tryLoftContactDirectShellUnion(t.Context(),
					meshbool.OpUnion, d, d.nextProducerID(), first, second)
				require.NoError(t, err)
				require.True(t, admitted, "the public trapezoid must reach the direct shell")
				require.NotNil(t, candidate)
				if !reverse {
					before := append([]*Body(nil), d.Bodies()...)
					beforeProducer := d.nextProducerID()
					limited := context.WithValue(t.Context(), loftDirectOutputLimitKey{}, uint64(1))
					refused, refusalErr := Union(limited, first, second)
					require.True(t, refused == nil)
					require.ErrorIs(t, refusalErr, ErrUnsupported)
					require.ErrorContains(t, refusalErr, "direct Loft/prism bounded proof limit")
					after := d.Bodies()
					require.Equal(t, len(before), len(after), "the refusal must not commit a body")
					for i := range before {
						require.True(t, before[i] == after[i], "body %d changed after refusal", i)
					}
					require.Equal(t, beforeProducer, d.nextProducerID(),
						"the refusal must not advance the document producer")
				}
				result, err := Union(t.Context(), first, second)
				require.NoError(t, err)
				require.True(t, result.IsSolid())
				volume, err := result.Volume()
				require.NoError(t, err)
				require.LessOrEqual(t, math.Abs(volume.Value.Base()-524), volume.Bound.Base())
				for _, role := range []string{"side(0,0,0)", "side(0,0,1)"} {
					ref := featureRefOf(provenance.Ref{Producer: loft.n.Origin.Producer, Role: role})
					faces, selectErr := Faces(FaceCreatedBy(ref)).SelectFaces(result)
					require.NoError(t, selectErr)
					require.Greater(t, len(faces), 0, "missing Loft source role %q", role)
				}
				for _, edge := range result.Edges() {
					require.Equal(t, 2, len(edge.Faces()))
				}
				report, err := d.Verify(t.Context())
				require.NoError(t, err)
				require.True(t, report.Passed(), "status=%v bodies=%d", report.Status, len(report.Bodies))
			})
		}
	})
}

func TestLoftContactDirectShellUnionRealVaryingTab(t *testing.T) {
	ctx := t.Context()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.Vec{Y: 1},
		r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	base, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	upper, err := world.CreateOffsetPlane(base, 4)
	require.NoError(t, err)
	var sections [2]*sketch.Sketch
	var profiles [2]*sketch.Profile
	for i, coords := range [2][4][2]float64{
		{{8, 4}, {12, 4}, {12, 6}, {8, 6}},
		{{8, 4}, {13.5, 4}, {13.5, 6}, {8, 6}},
	} {
		plane := base
		if i == 1 {
			plane = upper
		}
		s, createErr := world.CreateSketch(plane)
		require.NoError(t, createErr)
		var points [4]*sketch.Point
		for j, xy := range coords {
			points[j] = s.CreatePoint(xy[0], xy[1])
		}
		for j := range points {
			s.CreateLine(points[j], points[(j+1)%len(points)])
		}
		s.Fix(points[0])
		_, createErr = s.Solve(ctx)
		require.NoError(t, createErr)
		sections[i], profiles[i] = s, s.Profiles()[0]
	}
	d := New()
	loft, err := Loft(ctx, d, sections[0], profiles[0], sections[1], profiles[1])
	require.NoError(t, err)
	s, err := world.CreateSketch(base)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	prism, err := Extrude(ctx, d, s, s.Profiles()[0],
		Distance{D: units.Millimeters(5), Dir: Along})
	require.NoError(t, err)
	pp, lp, boundary, operation := loftVolumeRealCertificates(t, prism, loft)
	proof, admitted, err := loftVolumeCompose(ctx, operation, pp, lp, boundary)
	require.NoError(t, err)
	require.True(t, admitted)
	require.Zero(t, proof.sourceVolume.Cmp(big.NewRat(522, 1)))
	result, err := Union(ctx, prism, loft)
	require.NoError(t, err)
	require.True(t, result.IsSolid())
	volume, err := result.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-522), volume.Bound.Base(),
		"measured=%g bound=%g", volume.Value.Base(), volume.Bound.Base())
	roles := map[string]bool{}
	sharedWallFace := false
	for _, face := range result.Faces() {
		faceRoles := map[string]bool{}
		for _, ref := range face.originRefs() {
			if ref.Producer == loft.n.Origin.Producer {
				roles[ref.Role] = true
				faceRoles[ref.Role] = true
			}
		}
		sharedWallFace = sharedWallFace ||
			faceRoles["side(0,0,0)"] && faceRoles["side(0,0,1)"]
	}
	require.True(t, roles["side(0,0,0)"], "first crossing-wall triangle is absent")
	require.True(t, roles["side(0,0,1)"], "second crossing-wall triangle is absent")
	require.True(t, roles["side(0,1,0)"], "first Loft wall triangle is absent")
	require.True(t, roles["side(0,1,1)"], "second Loft wall triangle is absent")
	require.True(t, sharedWallFace, "the crossing wall lacks both contributing sources")
	for _, edge := range result.Edges() {
		require.Equal(t, 2, len(edge.Faces()))
	}
}
