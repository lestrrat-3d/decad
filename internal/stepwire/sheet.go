package stepwire

import (
	"context"
	"encoding/binary"
	"fmt"
	"io"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/stepbrep"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func writeSheet(ctx context.Context, w io.Writer, body *decad.Body, tolerance units.Value) error {
	mesh, err := body.Tessellate(ctx, tolerance, decad.WithVerification(decad.VerifyBoundary))
	if err != nil {
		return err
	}
	if !mesh.BoundaryVerified() {
		return fmt.Errorf("stepwire: %w: unverified sheet", decad.ErrUnsupported)
	}
	vertices, triangles := mesh.Vertices(), mesh.Triangles()
	nv, nt := uint64(len(vertices)), uint64(len(triangles))
	if nv == 0 || nt == 0 || nv > uint64(maxBytes-16)/24 || nt > maxTriangles ||
		nt > (uint64(maxBytes)-16-24*nv)/12 || nv > uint64(math.MaxUint32)+1 {
		return fmt.Errorf("stepwire: %w: sheet size", decad.ErrMeshTooLarge)
	}
	bound, err := mesh.Bound().In(units.Millimeter)
	if err != nil || math.IsNaN(bound) || math.IsInf(bound, 0) || bound < 0 {
		return fmt.Errorf("stepwire: %w: sheet bound", decad.ErrUnsupported)
	}
	data := make([]byte, int(16+24*nv+12*nt))
	binary.LittleEndian.PutUint64(data, nv)
	binary.LittleEndian.PutUint64(data[8:], nt)
	pos := 16
	for i, v := range vertices {
		if i%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		for _, value := range [...]float64{v.X, v.Y, v.Z} {
			if math.IsNaN(value) || math.IsInf(value, 0) {
				return fmt.Errorf("stepwire: %w: sheet coordinate", decad.ErrNotFinite)
			}
			binary.LittleEndian.PutUint64(data[pos:], math.Float64bits(value))
			pos += 8
		}
	}
	for i, tri := range triangles {
		if i%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		for _, index := range tri {
			if index < 0 || uint64(index) >= nv || uint64(index) > math.MaxUint32 {
				return fmt.Errorf("stepwire: %w: sheet index", decad.ErrUnsupported)
			}
			binary.LittleEndian.PutUint32(data[pos:], uint32(index))
			pos += 4
		}
	}
	return writeFrame(ctx, w, "DCS3", data, bound)
}

func decodeSheet(ctx context.Context, data []byte) (stepbrep.Mesh, error) {
	bad := func() (stepbrep.Mesh, error) {
		return stepbrep.Mesh{}, fmt.Errorf("stepwire: %w: sheet data", decad.ErrMalformedMesh)
	}
	if len(data) < 16 {
		return bad()
	}
	nv, nt := binary.LittleEndian.Uint64(data), binary.LittleEndian.Uint64(data[8:])
	if nv == 0 || nt == 0 || nv > uint64(maxBytes-16)/24 || nt > maxTriangles ||
		nt > (uint64(maxBytes)-16-24*nv)/12 || nv > uint64(math.MaxUint32)+1 ||
		16+24*nv+12*nt != uint64(len(data)) {
		return bad()
	}
	mesh := stepbrep.Mesh{Vertices: make([]r3.Vec, int(nv)), Triangles: make([][3]int, int(nt))}
	pos := 16
	for i := range mesh.Vertices {
		if i%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return stepbrep.Mesh{}, err
			}
		}
		var xyz [3]float64
		for j := range xyz {
			xyz[j] = math.Float64frombits(binary.LittleEndian.Uint64(data[pos:]))
			pos += 8
			if math.IsNaN(xyz[j]) || math.IsInf(xyz[j], 0) {
				return bad()
			}
		}
		mesh.Vertices[i] = r3.NewVec(xyz[0], xyz[1], xyz[2])
	}
	for i := range mesh.Triangles {
		if i%4096 == 0 {
			if err := ctx.Err(); err != nil {
				return stepbrep.Mesh{}, err
			}
		}
		for j := range mesh.Triangles[i] {
			index := binary.LittleEndian.Uint32(data[pos:])
			pos += 4
			if uint64(index) >= nv {
				return bad()
			}
			mesh.Triangles[i][j] = int(index)
		}
		tri := mesh.Triangles[i]
		if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] {
			return bad()
		}
	}
	return mesh, nil
}
