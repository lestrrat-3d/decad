package linkage_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/linkage"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The model is generic over its body reference, so integers stand for bodies
// here and no document is needed.
func TestLinkageModelOverIntegerBodies(t *testing.T) {
	t.Parallel()
	l := linkage.NewLinkage[int]()
	require.Empty(t, l.Links())

	arm, err := l.Ground().Revolute(r3.Vec{}, r3.Vec{Z: 1}, []int{1, 2})
	require.NoError(t, err)
	slide, err := arm.Prismatic(r3.Vec{X: 1}, []int{3})
	require.NoError(t, err)
	require.Equal(t, []*linkage.Link[int]{arm, slide}, l.Links())
	require.Same(t, arm, slide.Parent())
	require.Equal(t, []int{1, 2}, arm.Bodies())
	require.IsType(t, linkage.RevoluteJoint{}, arm.Joint())
	require.IsType(t, linkage.PrismaticJoint{}, slide.Joint())

	t.Run("a body belongs to one link", func(t *testing.T) {
		_, err := arm.Revolute(r3.Vec{}, r3.Vec{Z: 1}, []int{2})
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		_, err = arm.Revolute(r3.Vec{}, r3.Vec{Z: 1}, []int{9, 9})
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		_, err = arm.Revolute(r3.Vec{}, r3.Vec{Z: 1}, nil)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
	})
	t.Run("a joint needs a direction", func(t *testing.T) {
		_, err := arm.Revolute(r3.Vec{}, r3.Vec{}, []int{7})
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		_, err = arm.Prismatic(r3.Vec{}, []int{7})
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
	})
	t.Run("joint contacts name bodies of different links", func(t *testing.T) {
		require.ErrorIs(t, l.DeclareJointContact(1, 2), decaderr.ErrDegenerate, "one link")
		require.ErrorIs(t, l.DeclareJointContact(1, 1), decaderr.ErrDegenerate, "one body")
		require.ErrorIs(t, l.DeclareJointContact(0, 1), decaderr.ErrDegenerate, "a zero reference is no body")
		require.ErrorIs(t, l.DeclareJointContact(8, 9), decaderr.ErrDegenerate, "no link")
		require.NoError(t, l.DeclareJointContact(1, 3))
		require.ErrorIs(t, l.DeclareJointContact(3, 1), decaderr.ErrDegenerate, "declared twice")
		contacts := l.JointContacts()
		require.Len(t, contacts, 1)
		require.Equal(t, 1, contacts[0].A)
		require.Equal(t, 3, contacts[0].B)
	})
}

func TestLinkagePoseAtOverIntegerBodies(t *testing.T) {
	t.Parallel()
	l := linkage.NewLinkage[int]()
	arm, err := l.Ground().Revolute(r3.Vec{}, r3.Vec{Z: 1}, []int{1}, linkage.WithJointLimits(units.Degrees(-10), units.Degrees(100)))
	require.NoError(t, err)
	slide, err := arm.Prismatic(r3.Vec{X: 1}, []int{2})
	require.NoError(t, err)

	drive := linkage.Drive[int]{
		{Link: arm, From: units.Degrees(0), To: units.Degrees(90)},
		{Link: slide, From: units.Millimeters(0), To: units.Millimeters(10)},
	}
	pose, err := l.PoseAt(drive, units.Scalar(1))
	require.NoError(t, err)
	require.Len(t, pose.Poses, 2)
	require.InDelta(t, 90, pose.Values[0].Base()*180/3.141592653589793, 1e-9)
	// The arm has turned a quarter and the slide has run 10 mm along its own
	// x axis, which that turn points along world y.
	moved := pose.Poses[1].Apply(r3.Vec{})
	require.InDelta(t, 0, moved.X, 1e-9)
	require.InDelta(t, 10, moved.Y, 1e-9)

	start, err := l.PoseAt(drive, units.Scalar(0))
	require.NoError(t, err)
	origin := start.Poses[1].Apply(r3.Vec{})
	require.InDelta(t, 0, origin.X, 1e-12)
	require.InDelta(t, 0, origin.Y, 1e-12)
	require.InDelta(t, 0, origin.Z, 1e-12)

	_, err = l.PoseAt(drive, units.Millimeters(1))
	require.ErrorIs(t, err, decaderr.ErrUnitKind, "the fraction is dimensionless")

	outside := linkage.Drive[int]{{Link: arm, From: units.Degrees(0), To: units.Degrees(120)}}
	_, err = l.PoseAt(outside, units.Scalar(1))
	require.ErrorIs(t, err, decaderr.ErrDegenerate, "a waypoint outside the declared limits")
}
