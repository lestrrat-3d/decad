package report_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/seal"
	"github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// shift returns a pure translation by v.
func shift(v r3.Vec) r3.Transform {
	t, err := r3.Translation(v)
	if err != nil {
		panic(err)
	}
	return t
}

// stubReplay answers each replay question with a fixed, distinguishable value.
type stubReplay struct {
	present bool
	affine  bool
	at      units.Value
}

func (s *stubReplay) Present() bool        { return s != nil && s.present }
func (s *stubReplay) HasAffineProof() bool { return s.affine }

func (s *stubReplay) PosesAt(elapsed units.Value) (r3.Transform, r3.Transform, error) {
	s.at = elapsed
	return shift(r3.Vec{X: 1}), shift(r3.Vec{X: 2}), nil
}

func (s *stubReplay) PosesAtInterval(time, _, _ units.Value) (r3.Transform, r3.Transform, error) {
	s.at = time
	return shift(r3.Vec{Y: 1}), shift(r3.Vec{Y: 2}), nil
}

type sweepReport = report.SweepReportOf[int, int, int, int, *stubReplay]

func TestSweepReportWithoutReplayProof(t *testing.T) {
	t.Parallel()
	var nilReport *sweepReport
	require.False(t, nilReport.HasAffineReplayProof())
	_, _, err := nilReport.CertifiedPosesAt(units.Seconds(1))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	r := &sweepReport{}
	require.False(t, r.HasAffineReplayProof(), "a report without a proof has no affine proof")
	_, _, err = r.CertifiedPosesAt(units.Seconds(1))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	_, _, err = r.CertifiedPosesAtInterval(units.Seconds(1), units.Seconds(0), units.Seconds(2))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	r.SetReplay(seal.Key{}, &stubReplay{present: false, affine: true})
	require.False(t, r.HasAffineReplayProof(), "an absent proof is never affine")
}

func TestSweepReportDelegatesToReplayProof(t *testing.T) {
	t.Parallel()
	proof := &stubReplay{present: true, affine: true}
	r := &sweepReport{}
	r.SetReplay(seal.Key{}, proof)
	require.Same(t, proof, r.Replay(seal.Key{}))
	require.True(t, r.HasAffineReplayProof())

	a, b, err := r.CertifiedPosesAt(units.Seconds(3))
	require.NoError(t, err)
	require.Equal(t, shift(r3.Vec{X: 1}), a)
	require.Equal(t, shift(r3.Vec{X: 2}), b)
	require.InDelta(t, 3, proof.at.Base(), 0)

	a, b, err = r.CertifiedPosesAtInterval(units.Seconds(4), units.Seconds(0), units.Seconds(8))
	require.NoError(t, err)
	require.Equal(t, shift(r3.Vec{Y: 1}), a)
	require.Equal(t, shift(r3.Vec{Y: 2}), b)
	require.InDelta(t, 4, proof.at.Base(), 0)

	proof.affine = false
	require.False(t, r.HasAffineReplayProof(), "the answer follows the proof")
}

func TestSweepReportBracketEndsAtDuration(t *testing.T) {
	t.Parallel()
	var nilReport *sweepReport
	require.False(t, nilReport.BracketEndsAtDuration())

	r := &sweepReport{}
	require.False(t, r.BracketEndsAtDuration(), "no bracket")
	r.Bracket = &report.SweepInterval{}
	require.False(t, r.BracketEndsAtDuration(), "no exact right end")
	r.SetBracketRight(seal.Key{}, big.NewRat(1, 2))
	require.Equal(t, big.NewRat(1, 2), r.BracketRight(seal.Key{}))
	require.False(t, r.BracketEndsAtDuration(), "the right end is not one")
	r.SetBracketRight(seal.Key{}, big.NewRat(2, 2))
	require.True(t, r.BracketEndsAtDuration())
	r.Bracket = nil
	require.False(t, r.BracketEndsAtDuration(), "the right end means nothing without a bracket")
}

func TestSweepSampleExactFraction(t *testing.T) {
	t.Parallel()
	s := &report.SweepSampleOf[int, int, int, int]{}
	require.Nil(t, s.ExactFraction(seal.Key{}))
	s.SetExactFraction(seal.Key{}, big.NewRat(3, 4))
	require.Equal(t, big.NewRat(3, 4), s.ExactFraction(seal.Key{}))
}

// stubTrack answers each track question with a fixed value, in the integer
// reference instantiation.
type stubTrack struct{}

func (stubTrack) Start() report.SweepInstant {
	return report.SweepInstant{Fraction: units.Scalar(0.25)}
}
func (stubTrack) End() report.SweepInstant { return report.SweepInstant{Fraction: units.Scalar(0.75)} }

func (stubTrack) Features() (report.ContactFeatureOf[int, int, int], report.ContactFeatureOf[int, int, int]) {
	return report.ContactFeatureOf[int, int, int]{Face: 1, Edge: 2}, report.ContactFeatureOf[int, int, int]{Face: 3, Vertex: 4}
}

func (stubTrack) Normal() report.VecMeasurement {
	return report.VecMeasurement{Value: r3.Vec{Z: 1}}
}

func (stubTrack) Band() *report.Measurement {
	return &report.Measurement{Value: units.Millimeters(0.5)}
}

func (stubTrack) BandAt(fraction units.Value) (*report.Measurement, error) {
	return &report.Measurement{Value: units.Millimeters(fraction.Base())}, nil
}

func (stubTrack) ManifoldAt(units.Value) (*report.ContactManifoldOf[int, int, int], error) {
	return &report.ContactManifoldOf[int, int, int]{Points: []report.ContactPointOf[int, int, int]{{FaceA: 1, FaceB: 3}}}, nil
}

func TestSweepContactTrackDelegates(t *testing.T) {
	t.Parallel()
	track := report.NewSweepContactTrack[int, int, int](seal.Key{}, stubTrack{}, "producer record")
	require.Equal(t, "producer record", track.Source(seal.Key{}))
	require.InDelta(t, 0.25, track.Start().Fraction.Base(), 0)
	require.InDelta(t, 0.75, track.End().Fraction.Base(), 0)
	a, b := track.Features()
	require.Equal(t, 1, a.Face)
	require.Equal(t, 4, b.Vertex)
	require.InDelta(t, 1, track.Normal().Value.Z, 0)
	require.InDelta(t, 0.5, track.Band().Value.Base(), 0)
	band, err := track.BandAt(units.Scalar(0.4))
	require.NoError(t, err)
	require.InDelta(t, 0.4, band.Value.Base(), 0)
	manifold, err := track.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.Len(t, manifold.Points, 1)
}

func TestLiftSweepContactTrack(t *testing.T) {
	t.Parallel()
	lift := refLift()
	var none *report.SweepContactTrackOf[int, int, int]
	require.Nil(t, report.LiftSweepContactTrack(seal.Key{}, none, lift), "a nil track stays nil")

	track := report.NewSweepContactTrack[int, int, int](seal.Key{}, stubTrack{}, "producer record")
	lifted := report.LiftSweepContactTrack(seal.Key{}, track, lift)
	require.Equal(t, "producer record", lifted.Source(seal.Key{}), "the producer record survives the lift")

	a, b := lifted.Features()
	require.Equal(t, report.ContactFeatureOf[string, string, string]{Face: "1", Edge: "2"}, a)
	require.Equal(t, report.ContactFeatureOf[string, string, string]{Face: "3", Vertex: "4"}, b)
	manifold, err := lifted.ManifoldAt(units.Scalar(0.5))
	require.NoError(t, err)
	require.Equal(t, "1", manifold.Points[0].FaceA)
	require.Equal(t, "3", manifold.Points[0].FaceB)
	require.InDelta(t, 0.25, lifted.Start().Fraction.Base(), 0)
	require.InDelta(t, 0.75, lifted.End().Fraction.Base(), 0)
	require.InDelta(t, 1, lifted.Normal().Value.Z, 0)
	require.InDelta(t, 0.5, lifted.Band().Value.Base(), 0)
	band, err := lifted.BandAt(units.Scalar(0.4))
	require.NoError(t, err)
	require.InDelta(t, 0.4, band.Value.Base(), 0)
}
