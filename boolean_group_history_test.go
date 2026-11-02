package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/capplane"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestGroupUnionInputsUsesPublicHistory(t *testing.T) {
	doc := New()
	world := sketch.NewWorld()
	hostSketch, hostErr := world.CreateSketch(world.XY())
	require.NoError(t, hostErr)
	rectangle := hostSketch.CreateRectangle(-10, -10, 15, 20)
	hostSketch.Fix(rectangle.A)
	holeCenter := hostSketch.CreatePoint(-5, -5)
	hostSketch.Fix(holeCenter)
	hostSketch.CreateCircle(holeCenter, 1)
	_, hostErr = hostSketch.Solve(t.Context())
	require.NoError(t, hostErr)
	var hostProfile *sketch.Profile
	for _, candidate := range hostSketch.Profiles() {
		if candidate.Valid && len(candidate.Holes) == 1 {
			hostProfile = candidate
		}
	}
	require.NotNil(t, hostProfile)
	host, hostErr := Extrude(t.Context(), doc, hostSketch, hostProfile, Distance{D: units.Millimeters(10), Dir: Along})
	require.NoError(t, hostErr)
	makeCircle := func(x float64) *Body {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		center := s.CreatePoint(x, 0)
		s.Fix(center)
		s.CreateCircle(center, 1)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		body, err := Extrude(t.Context(), doc, s, s.Profiles()[0], Distance{D: units.Millimeters(4), Dir: Along})
		require.NoError(t, err)
		return body
	}
	place := func(y float64) *Body {
		basis := r3.Basis{EX: r3.Vec{Y: -1}, EY: r3.Vec{X: 1}, EZ: r3.Vec{Z: 1}}
		transform, transformErr := r3.FromBasis(basis, r3.Vec{X: 3, Y: y, Z: 6})
		require.NoError(t, transformErr)
		placed, transformErr := Transform(t.Context(), makeCircle(0), transform)
		require.NoError(t, transformErr)
		return placed
	}
	first, second, third := place(0), place(5), place(10)
	group, err := Union(t.Context(), first, second, third)
	require.NoError(t, err)
	meter := proofbound.NewReplayTrialBudget()
	source, ok, err := groupUnionSourceSnapshot(t.Context(), group, meter)
	require.NoError(t, err)
	require.True(t, ok)
	inputs, boundOrigins, caps := source.Inputs, source.Owners, source.GroupCaps
	require.Len(t, inputs, 3)
	require.Same(t, first, inputs[0])
	require.Same(t, second, inputs[1])
	require.Same(t, third, inputs[2])
	t.Run("source snapshot budget", func(t *testing.T) {
		baseline := proofbound.NewReplayTrialBudget()
		snap, ok, err := groupUnionSourceSnapshot(t.Context(), group, baseline)
		require.NoError(t, err)
		require.True(t, ok)
		require.NotNil(t, snap)
		w, r := baseline.Usage()
		bytes := baseline.Bytes()
		t.Logf("source snapshot work=%d records=%d bytes=%d", w, r, bytes)
		for _, tc := range []struct {
			name    string
			w, r, b uint64
			pass    bool
		}{{replayTestCaseExact, w, r, bytes, true}, {replayTestCaseWorkShort, w - 1, r, bytes, false}, {replayTestCaseRecordsShort, w, r - 1, bytes, false}, {replayTestCaseBytesShort, w, r, bytes - 1, false}} {
			t.Run(tc.name, func(t *testing.T) {
				budget, err := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
				require.NoError(t, err)
				out, ok, err := groupUnionSourceSnapshot(t.Context(), group, budget)
				if tc.pass {
					require.NoError(t, err)
					require.True(t, ok)
					require.NotNil(t, out)
				} else {
					require.ErrorIs(t, err, ErrUnsupported)
					require.False(t, ok)
					require.Nil(t, out)
				}
			})
		}
	})
	gp := group.n.Payload.(stackedrecord.Payload)
	require.Greater(t, gp.SectionDelta, 0.0)
	slab := gp.Slabs[0]
	require.Len(t, boundOrigins, len(slab.Regions))
	groupPlacement := prismplacement.Operand{Frame: gp.Frame, Xform: gp.Xform, Z0: slab.Z0, Z1: slab.Z1, Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta, Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted}
	for region, owner := range boundOrigins {
		require.GreaterOrEqual(t, owner, 0)
		require.Less(t, owner, len(inputs))
		lp := inputs[owner].n.Payload.(prismrecord.Payload)
		matched, matchErr := prismboolean.ReplayWholeCircleMatch(t.Context(), slab.Regions[region], lp.Profile, groupPlacement, lp.PlacementOperand(), meter)
		require.NoError(t, matchErr)
		require.True(t, matched, "source owner for region %d", region)
	}
	leaf := inputs[0].n.Payload.(prismrecord.Payload)
	require.True(t, inputs[0].n.HasStrictPrismWalkSeal())
	leafWalk, leafWalkOK, leafWalkErr := prismrecord.ReplayPrismBuiltWalk(t.Context(), &inputs[0].n, leaf.Walks,
		0, 0, proofbound.NewReplayTrialBudget())
	require.NoError(t, leafWalkErr)
	require.True(t, leafWalkOK)
	require.Equal(t, leaf.Walks.Outer[0].Kind, leafWalk.Kind)
	t.Run("source walk seal budget and mutation", func(t *testing.T) {
		baseline := proofbound.NewReplayTrialBudget()
		_, ok, err := prismrecord.ReplayPrismBuiltWalk(t.Context(), &inputs[0].n, leaf.Walks, 0, 0, baseline)
		require.NoError(t, err)
		require.True(t, ok)
		work, records := baseline.Usage()
		bytes := baseline.Bytes()
		t.Logf("source walk work=%d records=%d bytes=%d", work, records, bytes)
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, work, records, bytes, true},
			{replayTestCaseWorkShort, work - 1, records, bytes, false},
			{replayTestCaseRecordsShort, work, records - 1, bytes, false},
			{replayTestCaseBytesShort, work, records, bytes - 1, false},
		} {
			t.Run(tc.name, func(t *testing.T) {
				budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				require.NoError(t, err)
				_, ok, err := prismrecord.ReplayPrismBuiltWalk(t.Context(), &inputs[0].n, leaf.Walks, 0, 0, budget)
				if tc.want {
					require.NoError(t, err)
					require.True(t, ok)
				} else {
					require.Error(t, err)
					require.False(t, ok)
				}
			})
		}
		original := leaf.Walks.Outer[0].StartV
		t.Cleanup(func() { leaf.Walks.Outer[0].StartV = original })
		if original == 0 {
			leaf.Walks.Outer[0].StartV = math.Copysign(0, -1)
			if math.Signbit(original) {
				leaf.Walks.Outer[0].StartV = 0
			}
		} else {
			leaf.Walks.Outer[0].StartV = math.Nextafter(original, math.Inf(1))
		}
		_, ok, err = prismrecord.ReplayPrismBuiltWalk(t.Context(), &inputs[0].n, leaf.Walks, 0, 0,
			proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("produced circle replay chord", func(t *testing.T) {
		run := func(ctx context.Context, budget *proofbound.ReplayBudget) (tessellation.ChordSamples[int], error) {
			sealed, err := inputs[0].n.ReplayPrismBuildPayload(ctx, leaf.Frame, leaf.Xform,
				leaf.Z0, leaf.Z1, leaf.Z0Delta, leaf.Z1Delta, leaf.SectionDelta,
				leaf.Z0Denoted, leaf.Z1Denoted,
				[]sectionrecord.AnalyticProfileView{leaf.Profile.AnalyticSealView()}, budget)
			if err != nil || !sealed {
				return tessellation.ChordSamples[int]{}, err
			}
			walk, ok, err := prismrecord.ReplayPrismBuiltWalk(ctx, &inputs[0].n, leaf.Walks, 0, 0, budget)
			if err != nil || !ok {
				return tessellation.ChordSamples[int]{}, err
			}
			return tessellation.ReplayChordLoop(ctx, leaf.Profile.Outer,
				[]survey2d.SegmentWalk{walk}, .12, leaf.Z1-leaf.Z0, budget)
		}
		trial := proofbound.NewReplayTrialBudget()
		got, err := run(t.Context(), trial)
		require.NoError(t, err)
		ordinary, err := tessellation.ChordLoop(t.Context(), leaf.Profile.Outer,
			.12, leaf.Z1-leaf.Z0, freeform.NewFreeformWork(), leaf.Walks, 0,
			func(w survey2d.SideWalk) (int, error) { return w.Segs[0], nil },
			stationbound.ChordStationBound)
		require.NoError(t, err)
		require.Len(t, got.Samples, len(ordinary.Samples))
		for i := range got.Samples {
			require.Equal(t, math.Float64bits(ordinary.Samples[i].U), math.Float64bits(got.Samples[i].U))
			require.Equal(t, math.Float64bits(ordinary.Samples[i].V), math.Float64bits(got.Samples[i].V))
			require.GreaterOrEqual(t, got.BoundOf[i].U, ordinary.BoundOf[i].U)
			require.GreaterOrEqual(t, got.BoundOf[i].V, ordinary.BoundOf[i].V)
		}
		work, records := trial.Usage()
		bytes := trial.Bytes()
		t.Logf("source-bound circle chord work=%d records=%d bytes=%d", work, records, bytes)
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, work, records, bytes, true},
			{replayTestCaseWorkShort, work - 1, records, bytes, false},
			{replayTestCaseRecordsShort, work, records - 1, bytes, false},
			{replayTestCaseBytesShort, work, records, bytes - 1, false},
		} {
			t.Run(tc.name, func(t *testing.T) {
				budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				require.NoError(t, err)
				out, err := run(t.Context(), budget)
				if tc.want {
					require.NoError(t, err)
					require.Len(t, out.Samples, len(got.Samples))
				} else {
					require.Error(t, err)
					require.Empty(t, out.Samples)
				}
			})
		}
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		out, err := run(ctx, proofbound.NewReplayTrialBudget())
		require.Error(t, err)
		require.Empty(t, out.Samples)
	})
	badProfile := slab.Regions[0]
	badProfile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), badProfile.Outer.Segments...)
	badCircle := badProfile.Outer.Segments[0].(sectionrecord.CircleSeg)
	badCircle.Center.U += 1
	badProfile.Outer.Segments[0] = badCircle
	match, mutationErr := prismboolean.ReplayWholeCircleMatch(t.Context(), badProfile, leaf.Profile, groupPlacement, leaf.PlacementOperand(), proofbound.NewReplayTrialBudget())
	require.NoError(t, mutationErr)
	require.False(t, match)
	require.Len(t, caps, 3)
	hostPayload := host.n.Payload.(prismrecord.Payload)
	require.True(t, host.n.HasStrictPrismWalkSeal())
	hostWalk, hostWalkOK, hostWalkErr := prismrecord.ReplayPrismBuiltWalk(t.Context(), &host.n, hostPayload.Walks,
		0, 0, proofbound.NewReplayTrialBudget())
	require.NoError(t, hostWalkErr)
	require.True(t, hostWalkOK)
	require.Equal(t, hostPayload.Walks.Outer[0].Kind, hostWalk.Kind)
	hostMap, hostMapOK, hostMapErr := prismplacement.ReplayParallelSignedMap(t.Context(), hostPayload.PlacementOperand(), groupPlacement, meter)
	require.NoError(t, hostMapErr)
	require.True(t, hostMapOK)
	top, blindOK, blindErr := prismplacement.ReplayBlindLevels(t.Context(), hostPayload.PlacementOperand(), groupPlacement, hostMap.Shift, meter)
	require.NoError(t, blindErr)
	require.True(t, blindOK)
	require.True(t, top)
	scene, sceneOK, sceneErr := prismcells.ReplayBlindGroupScene(t.Context(), hostPayload.Profile, slab.Regions, hostPayload.PlacementOperand(), groupPlacement, hostPayload.SectionDelta, gp.SectionDelta, meter)
	require.NoError(t, sceneErr)
	require.True(t, sceneOK)
	clearance, clearErr := prismcells.ReplayGroupSourceClearance(t.Context(), scene, meter)
	require.NoError(t, clearErr)
	require.True(t, clearance)
	cutSection, sectionOK, sectionErr := prismcells.ReplayBlindGroupSection(t.Context(), scene, meter)
	require.NoError(t, sectionErr)
	require.True(t, sectionOK)
	require.Len(t, cutSection.Profile.Holes, len(hostPayload.Profile.Holes)+len(slab.Regions))
	require.Len(t, cutSection.HoleRegions, len(slab.Regions))
	require.Greater(t, cutSection.SectionDelta, 0.0)
	stack, stackOrigins, stackOK, stackErr := stackedrecord.ReplayBlindGroupPayload(t.Context(), hostPayload, gp, scene, meter)
	require.NoError(t, stackErr)
	require.True(t, stackOK)
	require.Len(t, stack.Slabs, 2)
	require.Len(t, stack.Interfaces, 1)
	require.Len(t, stackOrigins, len(slab.Regions))
	require.Equal(t, 6.0, stack.Slabs[0].Z1)
	require.Equal(t, 6.0, stack.Slabs[1].Z0)
	require.Len(t, stack.Slabs[0].Regions[0].Holes, len(hostPayload.Profile.Holes))
	require.Len(t, stack.Slabs[1].Regions[0].Holes, len(hostPayload.Profile.Holes)+len(slab.Regions))
	require.Len(t, stack.Interfaces[0].LowerExposed, len(slab.Regions))
	require.NoError(t, stackedrecord.Falsify(t.Context(), stack.Record()))
	t.Run("private two slab geometry", func(t *testing.T) {
		node, log, builtOK, buildErr := stackedrecord.ReplayBlindGroupBuildWithSource(t.Context(),
			envOf(documentNode(doc), doc.nextProducerID()), stack, &host.n, gp, scene, meter)
		require.NoError(t, buildErr)
		require.True(t, builtOK)
		require.NotNil(t, log)
		built := bodyHandle(node)
		volume, volumeErr := built.Volume()
		require.NoError(t, volumeErr)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-(7500-22*math.Pi)), volume.Bound.Base())
		mesh, meshErr := built.Tessellate(t.Context(), units.Millimeters(.1), WithVerification(VerifyAll))
		require.NoError(t, meshErr)
		require.True(t, mesh.BoundaryVerified())
		require.True(t, mesh.VolumeVerified())
	})
	t.Run("private route meter and rollback", func(t *testing.T) {
		run := func(m *proofbound.ReplayBudget) (*Body, error) {
			if _, ok, err := groupUnionSourceSnapshot(t.Context(), group, m); err != nil || !ok {
				return nil, err
			}
			cert, ok, err := prismcells.ReplayBlindGroupScene(t.Context(), hostPayload.Profile, slab.Regions,
				hostPayload.PlacementOperand(), groupPlacement, hostPayload.SectionDelta, gp.SectionDelta, m)
			if err != nil || !ok {
				return nil, err
			}
			payload, _, ok, err := stackedrecord.ReplayBlindGroupPayload(t.Context(), hostPayload, gp, cert, m)
			if err != nil || !ok {
				return nil, err
			}
			node, log, ok, err := stackedrecord.ReplayBlindGroupBuildWithSource(t.Context(),
				envOf(documentNode(doc), doc.nextProducerID()), payload, &host.n, gp, cert, m)
			if err != nil || !ok {
				return nil, err
			}
			if log == nil {
				return nil, ErrUnsupported
			}
			return bodyHandle(node), nil
		}
		baseline := proofbound.NewReplayTrialBudget()
		built, err := run(baseline)
		require.NoError(t, err)
		require.NotNil(t, built)
		work, records := baseline.Usage()
		bytes := baseline.Bytes()
		t.Logf("partial route meter work=%d records=%d bytes=%d", work, records, bytes)
		for _, limit := range []struct {
			name    string
			w, r, b uint64
			pass    bool
		}{
			{replayTestCaseExact, work, records, bytes, true},
			{replayTestCaseWorkShort, work - 1, records, bytes, false},
			{replayTestCaseRecordsShort, work, records - 1, bytes, false},
			{replayTestCaseBytesShort, work, records, bytes - 1, false},
		} {
			t.Run(limit.name, func(t *testing.T) {
				short, err := proofbound.NewReplayBudgetLimits(limit.w, limit.r, limit.b)
				require.NoError(t, err)
				out, err := run(short)
				if limit.pass {
					require.NoError(t, err)
					require.NotNil(t, out)
				} else {
					require.ErrorIs(t, err, ErrUnsupported)
					require.Nil(t, out)
				}
				require.NoError(t, doc.requireLive(host))
				require.NoError(t, doc.requireLive(group))
			})
		}
		originalBounds := host.n.Bounds
		defer func() { host.n.Bounds = originalBounds }()
		host.n.Bounds.Min.X = math.Nextafter(originalBounds.Min.X, math.Inf(1))
		out, err := run(proofbound.NewReplayTrialBudget())
		require.ErrorIs(t, err, ErrUnsupported)
		require.Nil(t, out)
		host.n.Bounds = originalBounds
		require.NoError(t, doc.requireLive(host))
		require.NoError(t, doc.requireLive(group))
	})

	otherClear, otherErr := prismcells.ReplayGroupSourceClearance(t.Context(), scene, proofbound.NewReplayTrialBudget())
	require.NoError(t, otherErr)
	require.False(t, otherClear)
	t.Run("scene and clearance budget", func(t *testing.T) {
		run := func(b *proofbound.ReplayBudget) (bool, error) {
			cert, ok, err := prismcells.ReplayBlindGroupScene(t.Context(), hostPayload.Profile, slab.Regions, hostPayload.PlacementOperand(), groupPlacement, hostPayload.SectionDelta, gp.SectionDelta, b)
			if err != nil || !ok {
				return false, err
			}
			return prismcells.ReplayGroupSourceClearance(t.Context(), cert, b)
		}
		baseline := proofbound.NewReplayTrialBudget()
		ok, err := run(baseline)
		require.NoError(t, err)
		require.True(t, ok)
		w, r := baseline.Usage()
		bytes := baseline.Bytes()
		for _, tc := range []struct {
			name    string
			w, r, b uint64
			pass    bool
		}{{replayTestCaseExact, w, r, bytes, true}, {replayTestCaseWorkShort, w - 1, r, bytes, false}, {replayTestCaseRecordsShort, w, r - 1, bytes, false}, {replayTestCaseBytesShort, w, r, bytes - 1, false}} {
			t.Run(tc.name, func(t *testing.T) {
				budget, err := proofbound.NewReplayBudgetLimits(tc.w, tc.r, tc.b)
				require.NoError(t, err)
				ok, err := run(budget)
				if tc.pass {
					require.NoError(t, err)
					require.True(t, ok)
				} else {
					require.ErrorIs(t, err, ErrUnsupported)
					require.False(t, ok)
				}
			})
		}
	})
	touching := append([]momentinput.Profile(nil), slab.Regions...)
	touching[0].Outer.Segments = append([]sectionrecord.CurveSegment(nil), touching[0].Outer.Segments...)
	movedCircle := touching[0].Outer.Segments[0].(sectionrecord.CircleSeg)
	movedCircle.Center = sectionrecord.Point2{U: 5, V: -8}
	touching[0].Outer.Segments[0] = movedCircle
	badMeter := proofbound.NewReplayTrialBudget()
	badScene, badSceneOK, badSceneErr := prismcells.ReplayBlindGroupScene(t.Context(), hostPayload.Profile, touching, hostPayload.PlacementOperand(), groupPlacement, hostPayload.SectionDelta, gp.SectionDelta, badMeter)
	require.NoError(t, badSceneErr)
	if badSceneOK {
		clearance, clearErr = prismcells.ReplayGroupSourceClearance(t.Context(), badScene, badMeter)
		require.NoError(t, clearErr)
		require.False(t, clearance)
	}
	wideMeter := proofbound.NewReplayTrialBudget()
	wideScene, wideSceneOK, wideSceneErr := prismcells.ReplayBlindGroupScene(t.Context(), hostPayload.Profile, slab.Regions, hostPayload.PlacementOperand(), groupPlacement, hostPayload.SectionDelta, 100, wideMeter)
	require.NoError(t, wideSceneErr)
	require.False(t, wideSceneOK)
	require.Nil(t, wideScene)
	var hostTop *topo.Face
	for _, face := range topobuild.Faces(&host.n) {
		if len(face.Origins) == 1 && face.Origins[0].Role == "capEnd" {
			require.Nil(t, hostTop)
			hostTop = face
		}
	}
	require.NotNil(t, hostTop)
	hostSealOK, hostSealErr := prismboolean.ReplayPrismCapRecord(t.Context(), &host.n, hostTop, true, meter)
	require.NoError(t, hostSealErr)
	require.True(t, hostSealOK)

	t.Run("group cap budget", func(t *testing.T) {
		checkSourceProofLimits(t, func(m prismboolean.ReplayCapBudget) (bool, error) {
			_, ok, err := prismboolean.ReplayGroupCapStructure(t.Context(), &group.n, m)
			return ok, err
		})
	})
	t.Run("leaf cap budget", func(t *testing.T) {
		checkSourceProofLimits(t, func(m prismboolean.ReplayCapBudget) (bool, error) {
			_, ok, err := prismboolean.ReplayPrismCapStructure(t.Context(), &inputs[0].n, m)
			return ok, err
		})
	})
	t.Run("host cap budget", func(t *testing.T) {
		checkSourceProofLimits(t, func(m prismboolean.ReplayCapBudget) (bool, error) {
			return prismboolean.ReplayPrismCapRecord(t.Context(), &host.n, hostTop, true, m)
		})
	})
	t.Run("circle source budget", func(t *testing.T) {
		checkSourceProofLimits(t, func(m prismboolean.ReplayCapBudget) (bool, error) {
			return prismboolean.ReplayWholeCircleMatch(t.Context(), slab.Regions[0], leaf.Profile, groupPlacement, leaf.PlacementOperand(), m)
		})
	})
	forged := *hostTop
	var hostShell *topo.FaceShell
	var hostFaceIndex int
	for _, lump := range host.n.Lumps {
		for _, shell := range lump.Shells {
			for i, face := range shell.Faces {
				if face == hostTop {
					hostShell = shell
					hostFaceIndex = i
				}
			}
		}
	}
	require.NotNil(t, hostShell)
	hostShell.Faces[hostFaceIndex] = &forged
	forgedOK, forgedErr := prismboolean.ReplayPrismCapRecord(t.Context(), &host.n, &forged, true, proofbound.NewReplayTrialBudget())
	hostShell.Faces[hostFaceIndex] = hostTop
	require.NoError(t, forgedErr)
	require.False(t, forgedOK)
	originalCap := hostTop.CapRecord
	staleCap := *originalCap
	staleCap.ZDenoted = big.NewRat(11, 1)
	hostTop.CapRecord = &staleCap
	staleOK, staleErr := prismboolean.ReplayPrismCapRecord(t.Context(), &host.n, hostTop, true, proofbound.NewReplayTrialBudget())
	hostTop.CapRecord = originalCap
	require.NoError(t, staleErr)
	require.False(t, staleOK)
	originalPayload := host.n.Payload
	stalePayload := hostPayload
	stalePayload.Profile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), hostPayload.Profile.Outer.Segments...)
	firstLine := stalePayload.Profile.Outer.Segments[0].(sectionrecord.LineSeg)
	firstLine.End.U += 1
	stalePayload.Profile.Outer.Segments[0] = firstLine
	host.n.Payload = stalePayload
	staleOK, staleErr = prismboolean.ReplayPrismCapRecord(t.Context(), &host.n, hostTop, true, proofbound.NewReplayTrialBudget())
	host.n.Payload = originalPayload
	require.NoError(t, staleErr)
	require.False(t, staleOK)
	wallEdge := caps[0].Wall.Loops[0].CoEdges[0].Edge
	originalCurve := wallEdge.Curve
	changedCurve := originalCurve.(surfacegeom.Circle3)
	changedCurve.Radius = units.Millimeters(9)
	wallEdge.Curve = changedCurve
	_, mutatedOK, mutatedErr := prismboolean.ReplayGroupCapStructure(t.Context(), &group.n, proofbound.NewReplayTrialBudget())
	wallEdge.Curve = originalCurve
	require.NoError(t, mutatedErr)
	require.False(t, mutatedOK)
	originalPosition := wallEdge.Start.Position
	wallEdge.Start.Position.X++
	_, mutatedOK, mutatedErr = prismboolean.ReplayGroupCapStructure(t.Context(), &group.n, proofbound.NewReplayTrialBudget())
	wallEdge.Start.Position = originalPosition
	require.NoError(t, mutatedErr)
	require.False(t, mutatedOK)
	hostPlane, planeOK, planeErr := capplane.ReplayOfFace(t.Context(), hostTop, meter)
	require.NoError(t, planeErr)
	require.True(t, planeOK)
	for _, capFaces := range caps {
		toolPlane, toolOK, toolErr := capplane.ReplayOfFace(t.Context(), capFaces.End, meter)
		require.NoError(t, toolErr)
		require.True(t, toolOK)
		same, sameErr := capplane.ReplaySame(t.Context(), hostPlane, toolPlane, meter)
		require.NoError(t, sameErr)
		require.True(t, same)
	}

	t.Logf("cap structure verified for %d group regions", len(caps))
	workAll, recordsAll := meter.Usage()
	t.Logf("source proof work=%d records=%d bytes=%d sectionDelta=%g", workAll, recordsAll, meter.Bytes(), gp.SectionDelta)
	t.Run("whole arc source identity", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(-10, -10, 15, 20)
		s.Fix(rect.A)
		slot, err := s.CreateSlot(-5, -6, -5, -4, 1.1)
		require.NoError(t, err)
		s.Fix(slot.C1)
		s.Fix(slot.C2)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		var profile *sketch.Profile
		for _, candidate := range s.Profiles() {
			if candidate.Valid && len(candidate.Holes) == 1 {
				profile = candidate
			}
		}
		require.NotNil(t, profile)
		arcHost, err := Extrude(t.Context(), doc, s, profile, Distance{D: units.Millimeters(10), Dir: Along})
		require.NoError(t, err)
		arcPayload := arcHost.n.Payload.(prismrecord.Payload)
		arcMeter := proofbound.NewReplayTrialBudget()
		arcScene, arcSceneOK, err := prismcells.ReplayBlindGroupScene(t.Context(), arcPayload.Profile, slab.Regions, arcPayload.PlacementOperand(), groupPlacement, arcPayload.SectionDelta, gp.SectionDelta, arcMeter)
		require.NoError(t, err)
		require.True(t, arcSceneOK)
		arcClear, err := prismcells.ReplayGroupSourceClearance(t.Context(), arcScene, arcMeter)
		require.NoError(t, err)
		require.True(t, arcClear)
	})
	// Continue the original history-only budget assertions with a new meter.
	meter = proofbound.NewReplayTrialBudget()
	_, ok, err = groupUnionInputs(meter.WorkBudget(t.Context()), group)
	require.NoError(t, err)
	require.True(t, ok)
	work, records := meter.Usage()
	require.Equal(t, uint64(7), work)
	require.Equal(t, uint64(3), records)
	require.Equal(t, uint64(176), meter.Bytes())

	errLimit := errors.New("test budget exhausted")
	for _, limits := range []struct {
		name                 string
		work, records, bytes uint64
		pass                 bool
	}{
		{replayTestCaseExact, 7, 3, 176, true}, {replayTestCaseWorkShort, 6, 3, 176, false},
		{replayTestCaseRecordsShort, 7, 2, 176, false}, {replayTestCaseBytesShort, 7, 3, 175, false},
	} {
		t.Run(limits.name, func(t *testing.T) {
			remainingWork, remainingRecords, remainingBytes := limits.work, limits.records, limits.bytes
			budget := &proofbound.WorkBudget{
				ErrFn: t.Context().Err,
				StepFn: func() error {
					if remainingWork == 0 {
						return errLimit
					}
					remainingWork--
					return nil
				},
				ReserveFn: func(n uint64) error {
					if n > remainingRecords {
						return errLimit
					}
					remainingRecords -= n
					return nil
				},
				ReserveBytesFn: func(n uint64) error {
					if n > remainingBytes {
						return errLimit
					}
					remainingBytes -= n
					return nil
				},
			}
			got, accepted, gateErr := groupUnionInputs(budget, group)
			if limits.pass {
				require.NoError(t, gateErr)
				require.True(t, accepted)
				require.Len(t, got, 3)
				return
			}
			require.ErrorIs(t, gateErr, errLimit)
			require.False(t, accepted)
			require.Nil(t, got)
		})
	}

	// A two-body Union uses its own ordered public input role names.
	binaryFirst, binarySecond := makeCircle(20), makeCircle(25)
	binaryGroup, binaryErr := Union(t.Context(), binaryFirst, binarySecond)
	require.NoError(t, binaryErr)
	binaryInputs, binaryOK, binaryErr := groupUnionInputs(proofbound.NewReplayTrialBudget().WorkBudget(t.Context()), binaryGroup)
	require.NoError(t, binaryErr)
	require.True(t, binaryOK)
	require.Len(t, binaryInputs, 2)
	require.Same(t, binaryFirst, binaryInputs[0])
	require.Same(t, binarySecond, binaryInputs[1])

	// The snapshot rejects a changed source role and does not publish a slice.
	op := doc.producers[group]
	oldRole := op.inputs[1].Role
	op.inputs[1].Role = "target"
	inputs, ok, err = groupUnionInputs(proofbound.NewReplayTrialBudget().WorkBudget(t.Context()), group)
	op.inputs[1].Role = oldRole
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, inputs)
	oldBody := op.inputs[1].Body
	op.inputs[1].Body = op.inputs[0].Body
	inputs, ok, err = groupUnionInputs(proofbound.NewReplayTrialBudget().WorkBudget(t.Context()), group)
	op.inputs[1].Body = oldBody
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, inputs)

	fullMeter := proofbound.NewReplayTrialBudget()
	result, admitted, cutErr := groupBlindCutBodyWithBudget(t.Context(), host, group,
		doc.nextProducerID(), fullMeter)
	require.NoError(t, cutErr)
	require.True(t, admitted)
	fullWork, fullRecords := fullMeter.Usage()
	fullBytes := fullMeter.Bytes()
	t.Logf("private root route work=%d records=%d bytes=%d", fullWork, fullRecords, fullBytes)
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{replayTestCaseExact, fullWork, fullRecords, fullBytes, true},
		{replayTestCaseWorkShort, fullWork - 1, fullRecords, fullBytes, false},
		{replayTestCaseRecordsShort, fullWork, fullRecords - 1, fullBytes, false},
		{replayTestCaseBytesShort, fullWork, fullRecords, fullBytes - 1, false},
	} {
		t.Run("private root route/"+tc.name, func(t *testing.T) {
			limited, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			require.NoError(t, err)
			got, ok, err := groupBlindCutBodyWithBudget(t.Context(), host, group, doc.nextProducerID(), limited)
			if tc.want {
				require.NoError(t, err)
				require.True(t, ok)
				require.NotNil(t, got)
			} else {
				require.False(t, ok)
				require.Nil(t, got)
				var limit *proofbound.WorkLimitError
				require.ErrorAs(t, err, &limit)
			}
			require.NoError(t, doc.requireLive(host))
			require.NoError(t, doc.requireLive(group))
		})
	}
	t.Run("published witness and history", func(t *testing.T) {
		source, ok, err := groupUnionSourceSnapshot(t.Context(), group, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.True(t, ok)
		leaves := make([]*topo.Body, len(source.Inputs))
		for i, leaf := range source.Inputs {
			leaves[i] = &leaf.n
		}
		require.NotNil(t, result.blindProof)
		run := func(b *proofbound.ReplayBudget) (bool, error) {
			return result.blindProof.faces.Recheck(t.Context(), &result.n, &host.n, &group.n, leaves, b)
		}
		full := proofbound.NewReplayTrialBudget()
		ok, err = run(full)
		require.NoError(t, err)
		require.True(t, ok)
		work, records := full.Usage()
		bytes := full.Bytes()
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			pass                 bool
		}{
			{replayTestCaseExact, work, records, bytes, true}, {replayTestCaseWorkShort, work - 1, records, bytes, false},
			{replayTestCaseRecordsShort, work, records - 1, bytes, false}, {replayTestCaseBytesShort, work, records, bytes - 1, false},
		} {
			t.Run(tc.name, func(t *testing.T) {
				b, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				require.NoError(t, err)
				ok, err := run(b)
				if tc.pass {
					require.NoError(t, err)
					require.True(t, ok)
					return
				}
				var limit *proofbound.WorkLimitError
				require.ErrorAs(t, err, &limit)
				require.False(t, ok)
			})
		}
		face := result.n.Lumps[0].Shells[0].Faces[0]
		oldOrigin := face.Origins[0]
		face.Origins[0].Role = replayTestCaseChanged
		ok, err = run(proofbound.NewReplayTrialBudget())
		face.Origins[0] = oldOrigin
		require.NoError(t, err)
		require.False(t, ok)
		oldRef := face.Upstream[0]
		face.Upstream[0].Role = replayTestCaseChanged
		ok, err = run(proofbound.NewReplayTrialBudget())
		face.Upstream[0] = oldRef
		require.NoError(t, err)
		require.False(t, ok)
		face.Source = &provenance.SourcePatch{}
		ok, err = run(proofbound.NewReplayTrialBudget())
		face.Source = nil
		require.NoError(t, err)
		require.False(t, ok)
		ownerMeter := proofbound.NewReplayTrialBudget()
		ok, err = recheckBlindFaceOwners(t.Context(), &result.n, ownerMeter)
		require.NoError(t, err)
		require.True(t, ok)
		ownerWork, _ := ownerMeter.Usage()
		exactOwners, err := proofbound.NewReplayBudgetLimits(ownerWork, 1, 1)
		require.NoError(t, err)
		ok, err = recheckBlindFaceOwners(t.Context(), &result.n, exactOwners)
		require.NoError(t, err)
		require.True(t, ok)
		shortOwners, err := proofbound.NewReplayBudgetLimits(ownerWork-1, 1, 1)
		require.NoError(t, err)
		_, err = recheckBlindFaceOwners(t.Context(), &result.n, shortOwners)
		var ownerLimit *proofbound.WorkLimitError
		require.ErrorAs(t, err, &ownerLimit)
		oldOwner := face.Owner
		face.Owner = struct{}{}
		ok, err = recheckBlindFaceOwners(t.Context(), &result.n, proofbound.NewReplayTrialBudget())
		face.Owner = oldOwner
		require.NoError(t, err)
		require.False(t, ok)
		payload := result.n.Payload.(stackedrecord.Payload)
		oldRole := payload.Upstream[0].Roles[0]
		payload.Upstream[0].Roles[0] = replayTestCaseChanged
		ok, err = run(proofbound.NewReplayTrialBudget())
		payload.Upstream[0].Roles[0] = oldRole
		require.NoError(t, err)
		require.False(t, ok)
		oldTableRef := payload.Upstream[0].Origins[0]
		payload.Upstream[0].Origins[0].Role = replayTestCaseChanged
		ok, err = run(proofbound.NewReplayTrialBudget())
		payload.Upstream[0].Origins[0] = oldTableRef
		require.NoError(t, err)
		require.False(t, ok)
		sourceFace := host.n.Lumps[0].Shells[0].Faces[0]
		oldSourceOrigin := sourceFace.Origins[0]
		sourceFace.Origins[0].Role = replayTestCaseChanged
		ok, err = run(proofbound.NewReplayTrialBudget())
		sourceFace.Origins[0] = oldSourceOrigin
		require.NoError(t, err)
		require.False(t, ok)
		leaves[0], leaves[1] = leaves[1], leaves[0]
		ok, err = run(proofbound.NewReplayTrialBudget())
		leaves[0], leaves[1] = leaves[1], leaves[0]
		require.NoError(t, err)
		require.False(t, ok)
		original := result.n.Lumps[0].Shells[0].Faces[0]
		result.n.Lumps[0].Shells[0].Faces[0] = result.n.Lumps[0].Shells[0].Faces[1]
		ok, err = run(proofbound.NewReplayTrialBudget())
		result.n.Lumps[0].Shells[0].Faces[0] = original
		require.NoError(t, err)
		require.False(t, ok)
		canceled, cancel := context.WithCancel(t.Context())
		cancel()
		_, err = result.blindProof.faces.Recheck(canceled, &result.n, &host.n, &group.n, leaves, proofbound.NewReplayTrialBudget())
		require.ErrorIs(t, err, context.Canceled)
		cut := &Operation{kind: OperationCut, doc: &doc.n, outputs: []*Body{result}, inputs: []OperationInput{
			{Body: host, Role: "target", Use: InputConsumed}, {Body: group, Role: "tool", Use: InputConsumed},
		}}
		require.True(t, result.blindProof.acceptsOperation(cut, result))
		result.blindProof.cut = cut
		doc.producers[result] = cut
		defer func() { delete(doc.producers, result); result.blindProof.cut = nil }()
		historyMeter := proofbound.NewReplayTrialBudget()
		ok, err = result.blindProof.recheckHistory(t.Context(), result, host, group, cut, historyMeter)
		require.NoError(t, err)
		require.True(t, ok)
		historyWork, _ := historyMeter.Usage()
		exactHistory, err := proofbound.NewReplayBudgetLimits(historyWork, 1, 1)
		require.NoError(t, err)
		ok, err = result.blindProof.recheckHistory(t.Context(), result, host, group, cut, exactHistory)
		require.NoError(t, err)
		require.True(t, ok)
		shortHistory, err := proofbound.NewReplayBudgetLimits(historyWork-1, 1, 1)
		require.NoError(t, err)
		_, err = result.blindProof.recheckHistory(t.Context(), result, host, group, cut, shortHistory)
		var historyLimit *proofbound.WorkLimitError
		require.ErrorAs(t, err, &historyLimit)
		_, err = result.blindProof.recheckHistory(canceled, result, host, group, cut, proofbound.NewReplayTrialBudget())
		require.ErrorIs(t, err, context.Canceled)
		originalUnion := doc.producers[group]
		replacement := *originalUnion
		doc.producers[group] = &replacement
		ok, err = result.blindProof.recheckHistory(t.Context(), result, host, group, cut, proofbound.NewReplayTrialBudget())
		doc.producers[group] = originalUnion
		require.NoError(t, err)
		require.False(t, ok)
		oldHostOrigin := host.n.Origin
		host.n.Origin.Producer++
		ok, err = result.blindProof.recheckHistory(t.Context(), result, host, group, cut, proofbound.NewReplayTrialBudget())
		host.n.Origin = oldHostOrigin
		require.NoError(t, err)
		require.False(t, ok)
		oldGroupOrigin := group.n.Origin
		group.n.Origin.Producer++
		ok, err = result.blindProof.recheckHistory(t.Context(), result, host, group, cut, proofbound.NewReplayTrialBudget())
		group.n.Origin = oldGroupOrigin
		require.NoError(t, err)
		require.False(t, ok)
		replacementCut := *cut
		doc.producers[result] = &replacementCut
		ok, err = result.blindProof.recheckHistory(t.Context(), result, host, group, &replacementCut, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("cached mesh comparison", func(t *testing.T) {
		face := result.n.Lumps[0].Shells[0].Faces[0]
		base := &topo.Mesh{Vertices: []r3.Vec{{}, {X: 1}, {Y: 1}}, Triangles: [][3]int{{0, 1, 2}},
			Source: []*topo.Face{face}, FaceBound: map[*topo.Face]float64{face: .1}, VertexBound: []float64{.1, .1, .1},
			Bound: .1, CoordBound: .01, AreaSlack: .2, VolSymDiff: .3, BoundaryOK: true, SymDiffOK: true}
		copyMesh := func() *topo.Mesh {
			cloned := *base
			cloned.Vertices = append([]r3.Vec(nil), base.Vertices...)
			cloned.Triangles = append([][3]int(nil), base.Triangles...)
			cloned.Source = append([]*topo.Face(nil), base.Source...)
			cloned.VertexBound = append([]float64(nil), base.VertexBound...)
			cloned.FaceBound = map[*topo.Face]float64{face: .1}
			return &cloned
		}
		full := proofbound.NewReplayTrialBudget()
		equal, err := sameBlindMesh(t.Context(), base, copyMesh(), full)
		require.NoError(t, err)
		require.True(t, equal)
		work, _ := full.Usage()
		exact, err := proofbound.NewReplayBudgetLimits(work, 1, 1)
		require.NoError(t, err)
		equal, err = sameBlindMesh(t.Context(), base, copyMesh(), exact)
		require.NoError(t, err)
		require.True(t, equal)
		short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
		require.NoError(t, err)
		_, err = sameBlindMesh(t.Context(), base, copyMesh(), short)
		var limit *proofbound.WorkLimitError
		require.ErrorAs(t, err, &limit)
		for _, mutate := range []func(*topo.Mesh){
			func(m *topo.Mesh) { m.Vertices[0].X = 1 },
			func(m *topo.Mesh) { m.Triangles[0][0] = 1 },
			func(m *topo.Mesh) { m.Source[0] = nil },
			func(m *topo.Mesh) { m.FaceBound[face] = .2 },
			func(m *topo.Mesh) { m.VertexBound[0] = .2 },
			func(m *topo.Mesh) { m.BoundaryOK = false },
		} {
			changed := copyMesh()
			mutate(changed)
			equal, err = sameBlindMesh(t.Context(), base, changed, proofbound.NewReplayTrialBudget())
			require.NoError(t, err)
			require.False(t, equal)
		}
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err = sameBlindMesh(ctx, base, copyMesh(), proofbound.NewReplayTrialBudget())
		require.ErrorIs(t, err, context.Canceled)
	})
	resultPayload, stacked := result.n.Payload.(stackedrecord.Payload)
	require.True(t, stacked)
	require.True(t, result.n.HasStrictPrismBuildSeal())
	sealedProfiles := []sectionrecord.AnalyticProfileView{resultPayload.Slabs[0].Regions[0].AnalyticSealView(), resultPayload.Slabs[1].Regions[0].AnalyticSealView()}
	sealed, sealErr := result.n.ReplayPrismBuildPayload(t.Context(), resultPayload.Frame, resultPayload.Xform,
		resultPayload.Slabs[0].Z0, resultPayload.Slabs[1].Z1,
		resultPayload.Slabs[0].Z0Delta, resultPayload.Slabs[1].Z1Delta,
		resultPayload.SectionDelta, resultPayload.Slabs[0].Z0Denoted, resultPayload.Slabs[1].Z1Denoted,
		sealedProfiles, proofbound.NewReplayTrialBudget())
	require.NoError(t, sealErr)
	require.True(t, sealed)
	tessStructureMeter := proofbound.NewReplayTrialBudget()
	boundedColumns, boundedRows, boundedPatches, structured, structureErr :=
		stackedrecord.ReplayBlindTessStructure(t.Context(), &result.n, tessStructureMeter)
	require.NoError(t, structureErr)
	require.True(t, structured)
	ordinaryColumns, ordinaryRows, ordinaryErr := resultPayload.Columns()
	require.NoError(t, ordinaryErr)
	ordinaryPatches, ordinaryErr := resultPayload.InterfacePatches(ordinaryColumns, ordinaryRows, 0)
	require.NoError(t, ordinaryErr)
	require.Len(t, boundedColumns, len(ordinaryColumns))
	require.Len(t, boundedRows, len(ordinaryRows))
	require.Len(t, boundedPatches, len(ordinaryPatches))
	t.Run("produced blind output chord structure", func(t *testing.T) {
		run := func(ctx context.Context, budget *proofbound.ReplayBudget) (int, error) {
			chords, ok, err := stackedrecord.ReplayBlindChordsWithSource(ctx,
				&result.n, &host.n, .12, budget)
			if err != nil || !ok {
				return 0, err
			}
			total := 0
			for _, samples := range chords.Loops {
				total += len(samples.Samples)
			}
			return total, nil
		}
		trial := proofbound.NewReplayTrialBudget()
		count, err := run(t.Context(), trial)
		require.NoError(t, err)
		require.Positive(t, count)
		work, records := trial.Usage()
		bytes := trial.Bytes()
		t.Logf("blind output chord work=%d records=%d bytes=%d samples=%d", work, records, bytes, count)
		for _, tc := range []struct {
			name                 string
			work, records, bytes uint64
			want                 bool
		}{
			{replayTestCaseExact, work, records, bytes, true},
			{replayTestCaseWorkShort, work - 1, records, bytes, false},
			{replayTestCaseRecordsShort, work, records - 1, bytes, false},
			{replayTestCaseBytesShort, work, records, bytes - 1, false},
		} {
			t.Run(tc.name, func(t *testing.T) {
				budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
				require.NoError(t, err)
				got, err := run(t.Context(), budget)
				if tc.want {
					require.NoError(t, err)
					require.Equal(t, count, got)
				} else {
					require.Error(t, err)
					require.Zero(t, got)
				}
			})
		}
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		got, err := run(ctx, proofbound.NewReplayTrialBudget())
		require.Error(t, err)
		require.Zero(t, got)
	})
	tessWork, tessRecords := tessStructureMeter.Usage()
	tessBytes := tessStructureMeter.Bytes()
	t.Logf("bounded tess structure work=%d records=%d bytes=%d", tessWork, tessRecords, tessBytes)
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		want                 bool
	}{
		{replayTestCaseExact, tessWork, tessRecords, tessBytes, true},
		{replayTestCaseWorkShort, tessWork - 1, tessRecords, tessBytes, false},
		{replayTestCaseRecordsShort, tessWork, tessRecords - 1, tessBytes, false},
		{replayTestCaseBytesShort, tessWork, tessRecords, tessBytes - 1, false},
	} {
		t.Run("bounded tess structure/"+tc.name, func(t *testing.T) {
			limited, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			require.NoError(t, err)
			_, _, _, ok, err := stackedrecord.ReplayBlindTessStructure(t.Context(), &result.n, limited)
			if tc.want {
				require.NoError(t, err)
				require.True(t, ok)
			} else {
				require.False(t, ok)
			}
		})
	}
	changed := resultPayload
	changed.Slabs = append([]stackedrecord.Slab(nil), resultPayload.Slabs...)
	changed.Slabs[0].Z1++
	result.n.Payload = changed
	_, _, _, structured, structureErr = stackedrecord.ReplayBlindTessStructure(t.Context(), &result.n, //nolint:dogsled // Only refusal and its error matter here.
		proofbound.NewReplayTrialBudget())
	require.NoError(t, structureErr)
	require.False(t, structured)
	result.n.Payload = resultPayload
	require.NotEmpty(t, resultPayload.Upstream)
	boundFaces := 0
	for _, lump := range result.n.Lumps {
		for _, shell := range lump.Shells {
			for _, face := range shell.Faces {
				require.NotEmpty(t, face.Upstream)
				boundFaces++
			}
		}
	}
	require.Len(t, resultPayload.Upstream, boundFaces)
	require.True(t, doc.producers[result] == nil, "unexpected result producer")
	require.NoError(t, doc.requireLive(host))
	require.NoError(t, doc.requireLive(group))
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	inputs, ok, err = groupUnionInputs(proofbound.NewReplayTrialBudget().WorkBudget(ctx), group)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	require.Nil(t, inputs)
	t.Run("commit binds the witnessed Cut operation", func(t *testing.T) {
		bad := []OperationInput{{Body: host, Role: "wrong", Use: InputConsumed},
			{Body: group, Role: "tool", Use: InputConsumed}}
		beforeProducer := doc.nextProducerID()
		beforeOperations := len(doc.OperationHistory().Operations())
		err := doc.commitOperationIn(t.Context(), nil, OperationCut, bad, result, host, group)
		require.ErrorIs(t, err, ErrUnsupported)
		require.Equal(t, beforeProducer, doc.nextProducerID())
		require.Len(t, doc.OperationHistory().Operations(), beforeOperations)
		require.NoError(t, doc.requireLive(host))
		require.NoError(t, doc.requireLive(group))
		require.True(t, result.blindProof.cut == nil, "unexpected Cut binding")
		good := []OperationInput{{Body: host, Role: "target", Use: InputConsumed},
			{Body: group, Role: "tool", Use: InputConsumed}}
		require.NoError(t, doc.commitOperationIn(t.Context(), nil, OperationCut, good, result, host, group))
		cut := doc.producers[result]
		require.True(t, cut == result.blindProof.cut, "Cut binding changed")
		ok, err := result.blindProof.recheckHistory(t.Context(), result, host, group, cut, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.True(t, ok)
		cachedMesh := bornMesh(&Mesh{})
		entry := &topo.TessEntry{ChordBits: math.Float64bits(.12), Verify: int(VerifyAll), Mesh: &cachedMesh.n, Blind: true}
		result.blindProof.cache.Store(&blindTessBinding{body: result, operation: cut, entry: entry, mesh: &cachedMesh.n})
		result.n.Tess.Store(entry)
		changedFace := result.n.Lumps[0].Shells[0].Faces[0]
		changedFace.Source = &provenance.SourcePatch{}
		mesh, err := result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
		changedFace.Source = nil
		require.ErrorIs(t, err, ErrUnsupported)
		require.True(t, mesh == nil, "unexpected mesh")
		require.True(t, entry == result.n.Tess.Load(), "cache entry changed")
		oldOwner := changedFace.Owner
		changedFace.Owner = struct{}{}
		mesh, err = result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
		changedFace.Owner = oldOwner
		require.ErrorIs(t, err, ErrUnsupported)
		require.True(t, mesh == nil, "unexpected mesh")
		oldHostOrigin := host.n.Origin
		host.n.Origin.Producer++
		mesh, err = result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
		host.n.Origin = oldHostOrigin
		require.ErrorIs(t, err, ErrUnsupported)
		require.True(t, mesh == nil, "unexpected mesh")
		oldUnion := doc.producers[group]
		changedUnion := *oldUnion
		doc.producers[group] = &changedUnion
		mesh, err = result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
		doc.producers[group] = oldUnion
		require.ErrorIs(t, err, ErrUnsupported)
		require.True(t, mesh == nil, "unexpected mesh")
		changedCut := *cut
		doc.producers[result] = &changedCut
		mesh, err = result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
		doc.producers[result] = cut
		require.ErrorIs(t, err, ErrUnsupported)
		require.True(t, mesh == nil, "unexpected mesh")
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err = result.Tessellate(ctx, units.Millimeters(.12), WithVerification(VerifyAll))
		require.ErrorIs(t, err, context.Canceled)

		// A missing binding falls through to fresh replay on the same meter.
		result.blindProof.cache.Store(nil)
		freshBudget := proofbound.NewReplayTrialBudget()
		freshMesh, admitted, freshErr := groupBlindVerifyAllWithBudget(t.Context(), result, .12, freshBudget)
		freshWork, freshRecords := freshBudget.Usage()
		missBudget := proofbound.NewReplayTrialBudget()
		missMesh, missErr := tessellateContextWithBudget(t.Context(), result, units.Millimeters(.12), VerifyAll, missBudget)
		missWork, missRecords := missBudget.Usage()
		require.NoError(t, missErr)
		require.NoError(t, freshErr)
		require.True(t, admitted && freshMesh != nil && missMesh != nil)
		require.True(t, missWork > freshWork)
		require.True(t, missRecords > freshRecords)
		require.True(t, missBudget.Bytes() > freshBudget.Bytes())
		entry = result.n.Tess.Load()
		mesh, err = result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyBoundary))
		require.ErrorIs(t, err, ErrUnsupported)
		require.True(t, mesh == nil, "unexpected mesh")
		require.True(t, entry == result.n.Tess.Load(), "cache entry changed")

		payload := result.n.Payload.(stackedrecord.Payload)
		missingUpstream := payload
		missingUpstream.Upstream = nil
		result.n.Payload = missingUpstream
		defer func() { result.n.Payload = payload }()
		for _, cached := range []*topo.TessEntry{nil, entry} {
			result.n.Tess.Store(cached)
			mesh, err := result.Tessellate(t.Context(), units.Millimeters(.12), WithVerification(VerifyAll))
			require.ErrorIs(t, err, ErrUnsupported)
			require.True(t, mesh == nil, "unexpected mesh")
			require.True(t, cached == result.n.Tess.Load(), "cache entry changed")
		}
	})
}

func TestGroupDirectPrismSourceHistory(t *testing.T) {
	doc := New()
	source := internalBoxBody(t, doc, 0, 0, 4, 5, 6)
	placed, err := Transform(t.Context(), source, r3.Identity())
	if err != nil {
		t.Fatal(err)
	}
	meter := proofbound.NewReplayTrialBudget()
	ok, err := groupDirectPrismSource(t.Context(), placed, meter)
	if err != nil || !ok {
		t.Fatalf("transformed Extrude admitted=%t error=%v", ok, err)
	}
	w, r := meter.Usage()
	for _, tc := range []struct {
		name string
		work uint64
		want bool
	}{{replayTestCaseExact, w, true}, {replayTestCaseWorkShort, w - 1, false}} {
		t.Run(tc.name, func(t *testing.T) {
			limited, e := proofbound.NewReplayBudgetLimits(tc.work, max(r, 1), 1)
			if e != nil {
				t.Fatal(e)
			}
			admitted, e := groupDirectPrismSource(t.Context(), placed, limited)
			if tc.want && (e != nil || !admitted) || !tc.want && (e == nil || admitted) {
				t.Fatalf("history admitted=%t error=%v", admitted, e)
			}
		})
	}
	op := doc.producers[placed]
	oldRole := op.inputs[0].Role
	op.inputs[0].Role = "other"
	admitted, err := groupDirectPrismSource(t.Context(), placed, proofbound.NewReplayTrialBudget())
	op.inputs[0].Role = oldRole
	if err != nil || admitted {
		t.Fatalf("changed transform input admitted=%t error=%v", admitted, err)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	if admitted, err := groupDirectPrismSource(ctx, placed, proofbound.NewReplayTrialBudget()); !errors.Is(err, context.Canceled) || admitted {
		t.Fatalf("cancelled history admitted=%t error=%v", admitted, err)
	}
}

type sourceProofLimitMeter struct{ work, records, bytes, maxWork, maxRecords, maxBytes uint64 }

var errSourceProofLimit = errors.New("source proof limit")

func (m *sourceProofLimitMeter) SpendWork(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.maxWork-m.work {
		return errSourceProofLimit
	}
	m.work += n
	return nil
}
func (m *sourceProofLimitMeter) ReserveRecords(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.maxRecords-m.records {
		return errSourceProofLimit
	}
	m.records += n
	return nil
}
func (m *sourceProofLimitMeter) ReserveBytes(ctx context.Context, n uint64) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if n > m.maxBytes-m.bytes {
		return errSourceProofLimit
	}
	m.bytes += n
	return nil
}
func (m *sourceProofLimitMeter) ReserveArray(ctx context.Context, n, size uint64) error {
	if size != 0 && n > ^uint64(0)/size {
		return errSourceProofLimit
	}
	bytes := n * size
	if bytes > (^uint64(0)-128)/2 {
		return errSourceProofLimit
	}
	return m.ReserveBytes(ctx, 2*bytes+128)
}

func checkSourceProofLimits(t *testing.T, check func(prismboolean.ReplayCapBudget) (bool, error)) {
	t.Helper()
	baseline := proofbound.NewReplayTrialBudget()
	ok, err := check(baseline)
	require.NoError(t, err)
	require.True(t, ok)
	work, records := baseline.Usage()
	bytes := baseline.Bytes()
	for _, tc := range []struct {
		name    string
		w, r, b uint64
		pass    bool
	}{
		{replayTestCaseExact, work, records, bytes, true}, {replayTestCaseWorkShort, work - 1, records, bytes, false},
		{replayTestCaseRecordsShort, work, records - 1, bytes, false}, {replayTestCaseBytesShort, work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m := &sourceProofLimitMeter{maxWork: tc.w, maxRecords: tc.r, maxBytes: tc.b}
			ok, err := check(m)
			if tc.pass {
				require.NoError(t, err)
				require.True(t, ok)
				return
			}
			require.ErrorIs(t, err, errSourceProofLimit)
			require.False(t, ok)
		})
	}
}
