package tessellation

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

const replayTestCaseWorkShort = "work short"
const replayTestCaseRecordsShort = "records short"
const replayTestCaseBytesShort = "bytes short"

// BlindContactPairForTest exposes the unexported pair decision to the external
// reference test without adding a production API.
func BlindContactPairForTest(ctx context.Context, vertices []r3.Vec, tris [][3]int) (bool, error) {
	exact := make([]blindContactVector, len(vertices))
	for i, p := range vertices {
		for j, x := range [...]float64{p.X, p.Y, p.Z} {
			blindContactCoordinate(x, &exact[i][j])
		}
	}
	a := blindContactArithmetic{ctx, proofbound.NewReplayTrialBudget()}
	normals := make([]blindContactVector, 2)
	var scratch blindContactScratch
	for i, tri := range tris {
		if err := a.contactNormal(&exact[tri[0]], &exact[tri[1]], &exact[tri[2]], &normals[i], &scratch); err != nil {
			return false, err
		}
	}
	separated, err := contactVertexSeparated(ctx, a.budget, vertices, tris[0], tris[1])
	if err != nil || separated {
		return separated, err
	}
	return a.contactPair(exact, vertices, normals, tris[0], tris[1], 0, 1, &scratch)
}

func TestReplayRequireEmbeddedBoundary(t *testing.T) {
	for _, tc := range []struct {
		name  string
		x     float64
		valid bool
	}{
		{"lower", 0x1p-64, true},
		{"below lower", math.Nextafter(0x1p-64, 0), false},
		{"upper", math.Nextafter(0x1p32, 0), true},
		{"above upper", 0x1p32, false},
		{"positive zero", 0, true},
		{"negative zero", math.Copysign(0, -1), true},
		{"negative", -0x1.123456789abcdp31, true},
		{"subnormal", math.SmallestNonzeroFloat64, false},
		{"infinity", math.Inf(1), false},
		{"nan", math.NaN(), false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var got blindContactInt
			ok := blindContactCoordinate(tc.x, &got)
			require.Equal(t, tc.valid, ok)
			if !ok {
				return
			}
			requireBlindContactWidth(t, &got)
			value := new(big.Int)
			for i := len(got.limbs) - 1; i >= 0; i-- {
				value.Lsh(value, 64)
				value.Or(value, new(big.Int).SetUint64(got.limbs[i]))
			}
			if got.negative {
				value.Neg(value)
			}
			want := new(big.Rat).SetFloat64(tc.x)
			want.Mul(want, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 116)))
			require.True(t, want.IsInt())
			require.Zero(t, value.Cmp(want.Num()))
		})
	}
	budget := proofbound.NewReplayTrialBudget()
	err := ReplayRequireEmbeddedBoundary(t.Context(), []r3.Vec{{X: 1}}, nil, budget)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	work, records := budget.Usage()
	require.Equal(t, uint64(12), work)
	require.Zero(t, records)
	require.Zero(t, budget.Bytes())
	short, err := proofbound.NewReplayBudgetLimits(11, 1, 1)
	require.NoError(t, err)
	require.ErrorIs(t, ReplayRequireEmbeddedBoundary(t.Context(), []r3.Vec{{X: 1}}, nil, short), decaderr.ErrUnsupported)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.True(t, errors.Is(ReplayRequireEmbeddedBoundary(ctx, nil, nil, budget), context.Canceled))
}

func TestBlindContactBoxScanCharges(t *testing.T) {
	vertices := []r3.Vec{
		{X: 1}, {X: 2}, {X: 1, Y: 1},
		{X: 4}, {X: 5}, {X: 4, Y: 1},
	}
	triangles := [][3]int{{0, 1, 2}, {3, 4, 5}}
	budget := proofbound.NewReplayTrialBudget()
	require.NoError(t, ReplayRequireEmbeddedBoundary(t.Context(), vertices, triangles, budget))
	work, records := budget.Usage()
	require.Greater(t, work, uint64(126))
	require.Equal(t, uint64(11), records)
	require.Positive(t, budget.Bytes())

	short, err := proofbound.NewReplayBudgetLimits(77, 2, 1024)
	require.NoError(t, err)
	require.ErrorIs(t, ReplayRequireEmbeddedBoundary(t.Context(), vertices, triangles, short), decaderr.ErrUnsupported)
	work, records = short.Usage()
	require.Equal(t, uint64(72), work)
	require.Zero(t, records)
	require.Zero(t, short.Bytes())

	triangles[1][2] = len(vertices)
	require.ErrorIs(t, ReplayRequireEmbeddedBoundary(t.Context(), vertices, triangles,
		proofbound.NewReplayTrialBudget()), decaderr.ErrUnsupported)
}

func TestBlindContactScratchReservation(t *testing.T) {
	bytes := 2*(2*uint64(unsafe.Sizeof(blindContactBox{}))+8*uint64(unsafe.Sizeof(blindContactVector{}))+
		2*uint64(unsafe.Sizeof(blindContactScratch{}))) + 4*128 + 65536
	for _, tc := range []struct {
		name           string
		records, bytes uint64
		ok             bool
	}{
		{replayTestCaseExact, 9, bytes, true},
		{replayTestCaseBytesShort, 9, bytes - 1, false},
		{replayTestCaseRecordsShort, 8, bytes, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(1, tc.records, tc.bytes)
			require.NoError(t, err)
			err = reserveBlindContactStorage(t.Context(), budget, 6, 2)
			if tc.ok {
				require.NoError(t, err)
				_, records := budget.Usage()
				require.Equal(t, uint64(9), records)
				require.Equal(t, bytes, budget.Bytes())
			} else {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				_, records := budget.Usage()
				require.Zero(t, records)
				require.Zero(t, budget.Bytes())
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, reserveBlindContactStorage(ctx, proofbound.NewReplayTrialBudget(), 6, 2), context.Canceled)
}

func TestBlindContactScanReservation(t *testing.T) {
	for _, tc := range []struct {
		name          string
		work, records uint64
		ok            bool
	}{
		{replayTestCaseExact, 18, 3, true},
		{replayTestCaseWorkShort, 17, 3, false},
		{replayTestCaseRecordsShort, 18, 2, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, 1)
			require.NoError(t, err)
			err = reserveBlindContactScan(t.Context(), budget, 3)
			work, records := budget.Usage()
			if tc.ok {
				require.NoError(t, err)
				require.Equal(t, uint64(18), work)
				require.Equal(t, uint64(3), records)
			} else {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				require.Zero(t, work)
				require.Zero(t, records)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, reserveBlindContactScan(ctx, proofbound.NewReplayTrialBudget(), 3), context.Canceled)
}

func TestBlindContactPlanReservation(t *testing.T) {
	bytes := blindContactStorageBytes(6, 2)
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		ok                   bool
	}{
		{replayTestCaseExact, 6 + blindContactStackWork(), 11, bytes, true},
		{replayTestCaseWorkShort, 5 + blindContactStackWork(), 11, bytes, false},
		{replayTestCaseRecordsShort, 6 + blindContactStackWork(), 10, bytes, false},
		{replayTestCaseBytesShort, 6 + blindContactStackWork(), 11, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			require.NoError(t, err)
			err = reserveBlindContactPlan(t.Context(), budget, 6, 2)
			work, records := budget.Usage()
			if tc.ok {
				require.NoError(t, err)
				require.Equal(t, 6+blindContactStackWork(), work)
				require.Equal(t, uint64(11), records)
				require.Equal(t, bytes, budget.Bytes())
			} else {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
				require.Zero(t, work)
				require.Zero(t, records)
				require.Zero(t, budget.Bytes())
			}
		})
	}
}

func TestBlindContactSharedEdgePairWorkLimit(t *testing.T) {
	points := []r3.Vec{{}, {X: 2}, {Y: 2}, {Z: 2}}
	vertices := make([]blindContactVector, len(points))
	for i, point := range points {
		for j, x := range [...]float64{point.X, point.Y, point.Z} {
			blindContactCoordinate(x, &vertices[i][j])
		}
	}
	tris := [][3]int{{0, 1, 2}, {1, 0, 3}}
	normals := make([]blindContactVector, 2)
	var scratch blindContactScratch
	setup := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
	for i, tri := range tris {
		require.NoError(t, setup.contactNormal(&vertices[tri[0]], &vertices[tri[1]], &vertices[tri[2]], &normals[i], &scratch))
	}
	check := func(b *proofbound.ReplayBudget) (bool, error) {
		var pairScratch blindContactScratch
		return (blindContactArithmetic{t.Context(), b}).contactPair(vertices, points, normals, tris[0], tris[1], 0, 1, &pairScratch)
	}
	full := proofbound.NewReplayTrialBudget()
	got, err := check(full)
	require.NoError(t, err)
	require.True(t, got)
	work, _ := full.Usage()
	exact, err := proofbound.NewReplayBudgetLimits(work, 1, 1)
	require.NoError(t, err)
	got, err = check(exact)
	require.NoError(t, err)
	require.True(t, got)
	short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
	require.NoError(t, err)
	_, err = check(short)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
}

func TestBlindContactVertexSeparationWorkLimit(t *testing.T) {
	vertices := []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}, {X: -1}}
	x, y := [3]int{0, 1, 2}, [3]int{0, 3, 4}
	exact, err := proofbound.NewReplayBudgetLimits(160, 1, 1)
	require.NoError(t, err)
	got, err := contactVertexSeparated(t.Context(), exact, vertices, x, y)
	require.NoError(t, err)
	require.True(t, got)
	short, err := proofbound.NewReplayBudgetLimits(159, 1, 1)
	require.NoError(t, err)
	_, err = contactVertexSeparated(t.Context(), short, vertices, x, y)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	vertices[4].X = 1
	got, err = contactVertexSeparated(t.Context(), proofbound.NewReplayTrialBudget(), vertices, x, y)
	require.NoError(t, err)
	require.False(t, got)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = contactVertexSeparated(ctx, proofbound.NewReplayTrialBudget(), vertices, x, y)
	require.ErrorIs(t, err, context.Canceled)
}

func TestBlindContactRealTriangleIntervalsBothWindings(t *testing.T) {
	for _, tc := range []struct {
		x       float64
		allowed bool
	}{{1, false}, {4, true}} {
		for windingA := range 2 {
			for windingB := range 2 {
				points := [2][3]r3.Vec{
					{{}, {X: 4}, {Y: 4}},
					{{X: tc.x, Y: 1, Z: -1}, {X: tc.x, Y: 1, Z: 1}, {X: tc.x + 1, Y: 1}},
				}
				if windingA != 0 {
					points[0][1], points[0][2] = points[0][2], points[0][1]
				}
				if windingB != 0 {
					points[1][1], points[1][2] = points[1][2], points[1][1]
				}
				var p [2][3]blindContactVector
				for i := range p {
					for j, point := range points[i] {
						for k, value := range [...]float64{point.X, point.Y, point.Z} {
							ok := blindContactCoordinate(value, &p[i][j][k])
							require.True(t, ok)
						}
					}
				}
				a := blindContactArithmetic{t.Context(), proofbound.NewReplayTrialBudget()}
				var normals [2]blindContactVector
				for i := range p {
					u, err := a.difference(&p[i][1], &p[i][0])
					require.NoError(t, err)
					v, err := a.difference(&p[i][2], &p[i][0])
					require.NoError(t, err)
					normals[i], err = a.cross(&u, &v)
					require.NoError(t, err)
				}
				var endpoints [2][2][4]blindContactInt
				var products [2]blindContactInt
				var zero blindContactInt
				var counts [2]int
				for i := range p {
					var distances [3]blindContactInt
					for j := range distances {
						d, err := a.difference(&p[i][j], &p[1-i][0])
						require.NoError(t, err)
						distances[j], err = a.dot(&d, &normals[1-i])
						require.NoError(t, err)
					}
					var err error
					counts[i], err = a.clipContactTriangle(&p[i], &distances, &endpoints[i], &products, &zero)
					require.NoError(t, err)
				}
				direction, err := a.cross(&normals[0], &normals[1])
				require.NoError(t, err)
				var scratch blindContactScratch
				allowed, err := a.noncoplanarContactAllowed(&endpoints[0], &endpoints[1], counts[0], counts[1], 0, &direction, &scratch)
				require.NoError(t, err)
				require.Equal(t, tc.allowed, allowed)
			}
		}
	}
}
