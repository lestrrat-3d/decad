// Package stepwire implements bounded binary transport for read-only bodies.
package stepwire

import (
	"bytes"
	"compress/gzip"
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"math"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/decad/internal/payloadclass"
	"github.com/lestrrat-3d/decad/internal/stepbrep"
	"github.com/lestrrat-3d/decad/internal/stepread"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/units"
)

const analyticMagic = "DCS1"

const maxBytes = 64 << 20
const maxTriangles = 5_000_000

// Body retains received geometry until tessellation is requested.
type Body struct {
	model          *stepread.Model
	sheet          *stepbrep.Mesh
	sourceBound    float64
	hasSourceBound bool
}

// Write emits one frame, preferring analytic STEP and marking faceted output.
func Write(ctx context.Context, w io.Writer, body *decad.Body, verificationTolerance units.Value) error {
	if ctx == nil || w == nil || body == nil {
		return fmt.Errorf("stepwire: %w: nil argument", decad.ErrDegenerate)
	}
	chord, err := chordMM(verificationTolerance)
	if err != nil {
		return err
	}
	if body.Kind() == decad.BodySheet {
		return writeSheet(ctx, w, body, verificationTolerance)
	}
	header := step.Header{Description: []string{"decad body"}, Name: "body.step", Timestamp: time.Unix(0, 0).UTC(), Authors: []string{"decad"}, Organizations: []string{"decad"}}
	heldFacets := payloadclass.HoldsFacets(body)
	var file step.File
	if heldFacets {
		file, err = export.NewFacetedSTEPFile(ctx, body, verificationTolerance, header)
	} else {
		file, err = export.NewSTEPFile(ctx, body, verificationTolerance, header)
	}
	if err != nil {
		return err
	}
	product := ""
	for _, entity := range file.Entities {
		if entity.Name != "PRODUCT" || len(entity.Parameters) < 3 {
			continue
		}
		description, ok := entity.Parameters[2].(step.String)
		if !ok || product != "" {
			return fmt.Errorf("stepwire: %w: invalid STEP product", decad.ErrUnsupported)
		}
		product = string(description)
	}
	faceted := product == "faceted decad solid"
	if product != "analytic decad solid" && !faceted {
		return fmt.Errorf("stepwire: %w: unsupported STEP product", decad.ErrUnsupported)
	}
	data, err := receiveSTEP(file, chord)
	if err != nil && !faceted && errors.Is(err, decad.ErrUnsupported) {
		if err := ctx.Err(); err != nil {
			return err
		}
		file, err = export.NewFacetedSTEPFile(ctx, body, verificationTolerance, header)
		if err != nil {
			return err
		}
		faceted = true
		data, err = receiveSTEP(file, chord)
	}
	if err != nil {
		return err
	}
	magic := analyticMagic
	var sourceBound float64
	if faceted {
		mesh, err := body.Tessellate(ctx, verificationTolerance, decad.WithVerification(decad.VerifyBoundary))
		if err != nil {
			return err
		}
		sourceBound, err = mesh.Bound().In(units.Millimeter)
		if err != nil || math.IsNaN(sourceBound) || math.IsInf(sourceBound, 0) || sourceBound < 0 {
			return fmt.Errorf("stepwire: %w: invalid source bound", decad.ErrUnsupported)
		}
		magic = "DCS2"
	}
	return writeFrame(ctx, w, magic, data, sourceBound)
}

// receiveSTEP checks the real receiver before a STEP frame is sent.
func receiveSTEP(file step.File, chord float64) ([]byte, error) {
	data, err := file.Marshal()
	if err != nil {
		return nil, err
	}
	if len(data) > maxBytes {
		return nil, fmt.Errorf("stepwire: %w: STEP bytes", decad.ErrMeshTooLarge)
	}
	model, err := stepread.Parse(data)
	if err != nil {
		return nil, fmt.Errorf("stepwire: %w: %s", decad.ErrMalformedMesh, err)
	}
	if _, err := stepbrep.ReadAtTolerance(model, maxTriangles, chord); err != nil {
		return nil, err
	}
	return data, nil
}

func writeFrame(ctx context.Context, w io.Writer, magic string, data []byte, sourceBound float64) error {
	var compressed bytes.Buffer
	zw := gzip.NewWriter(&compressed)
	if _, err := zw.Write(data); err != nil {
		return err
	}
	if err := zw.Close(); err != nil {
		return err
	}
	if compressed.Len() > maxBytes {
		return fmt.Errorf("stepwire: %w: compressed bytes", decad.ErrMeshTooLarge)
	}
	headerBytes := 16
	if magic != analyticMagic {
		headerBytes = 24
	}
	frame := make([]byte, headerBytes+compressed.Len())
	copy(frame, magic)
	if magic != analyticMagic {
		binary.LittleEndian.PutUint64(frame[16:], math.Float64bits(sourceBound))
	}
	binary.LittleEndian.PutUint32(frame[4:], uint32(compressed.Len()))
	binary.LittleEndian.PutUint32(frame[8:], uint32(len(data)))
	copy(frame[headerBytes:], compressed.Bytes())
	if err := ctx.Err(); err != nil {
		return err
	}
	n, err := w.Write(frame)
	if err != nil {
		return err
	}
	if n != len(frame) {
		return io.ErrShortWrite
	}
	return nil
}

// Read consumes one frame and retains its geometry.
func Read(ctx context.Context, r io.Reader) (*Body, error) {
	if ctx == nil || r == nil {
		return nil, fmt.Errorf("stepwire: %w: nil argument", decad.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	var header [16]byte
	if _, err := io.ReadFull(r, header[:]); err != nil {
		return nil, err
	}
	clen, ulen := binary.LittleEndian.Uint32(header[4:]), binary.LittleEndian.Uint32(header[8:])
	magic := string(header[:4])
	if (magic != analyticMagic && magic != "DCS2" && magic != "DCS3") || clen == 0 || ulen == 0 || clen > maxBytes || ulen > maxBytes || binary.LittleEndian.Uint32(header[12:]) != 0 {
		return nil, fmt.Errorf("stepwire: %w: frame header", decad.ErrMalformedMesh)
	}
	var sourceBound float64
	if magic != analyticMagic {
		var b [8]byte
		if _, err := io.ReadFull(r, b[:]); err != nil {
			return nil, err
		}
		sourceBound = math.Float64frombits(binary.LittleEndian.Uint64(b[:]))
		if math.IsNaN(sourceBound) || math.IsInf(sourceBound, 0) || sourceBound < 0 {
			return nil, fmt.Errorf("stepwire: %w: source bound", decad.ErrMalformedMesh)
		}
	}
	compressed := make([]byte, int(clen))
	if _, err := io.ReadFull(r, compressed); err != nil {
		return nil, err
	}
	input := bytes.NewReader(compressed)
	zr, err := gzip.NewReader(input)
	if err != nil {
		return nil, fmt.Errorf("stepwire: %w: gzip header", decad.ErrMalformedMesh)
	}
	zr.Multistream(false)
	data, err := io.ReadAll(io.LimitReader(zr, int64(ulen)+1))
	if err != nil {
		return nil, fmt.Errorf("stepwire: %w: gzip data", decad.ErrMalformedMesh)
	}
	if err := zr.Close(); err != nil || len(data) != int(ulen) || input.Len() != 0 {
		return nil, fmt.Errorf("stepwire: %w: gzip length", decad.ErrMalformedMesh)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if magic == "DCS3" {
		mesh, err := decodeSheet(ctx, data)
		if err != nil {
			return nil, err
		}
		return &Body{sheet: &mesh, sourceBound: sourceBound, hasSourceBound: true}, nil
	}
	model, err := stepread.Parse(data)
	if err != nil {
		return nil, fmt.Errorf("stepwire: %w: %s", decad.ErrMalformedMesh, err)
	}
	return &Body{model: model, sourceBound: sourceBound, hasSourceBound: magic == "DCS2"}, nil
}

// IsSheet reports whether the source was a sheet body.
func (b *Body) IsSheet() bool { return b != nil && b.sheet != nil }

// SourceBoundaryBound returns the sender's claimed bound for faceted output.
func (b *Body) SourceBoundaryBound() (units.Value, bool) {
	if b == nil || !b.hasSourceBound {
		return units.Millimeters(0), false
	}
	return units.Millimeters(b.sourceBound), true
}

// Tessellate produces a mesh at the receiver's requested circular chord.
func (b *Body) Tessellate(ctx context.Context, tolerance units.Value) (stepbrep.Mesh, error) {
	if ctx == nil || b == nil || (b.model == nil && b.sheet == nil) {
		return stepbrep.Mesh{}, fmt.Errorf("stepwire: %w: nil argument", decad.ErrDegenerate)
	}
	if err := ctx.Err(); err != nil {
		return stepbrep.Mesh{}, err
	}
	chord, err := chordMM(tolerance)
	if err != nil {
		return stepbrep.Mesh{}, err
	}
	if b.hasSourceBound && chord < b.sourceBound {
		return stepbrep.Mesh{}, fmt.Errorf("stepwire: %w: tolerance below source bound", decad.ErrUnsupported)
	}
	if b.sheet != nil {
		return stepbrep.Mesh{Vertices: append([]r3.Vec(nil), b.sheet.Vertices...), Triangles: append([][3]int(nil), b.sheet.Triangles...)}, nil
	}
	mesh, err := stepbrep.ReadAtTolerance(b.model, maxTriangles, chord)
	if err != nil {
		return stepbrep.Mesh{}, err
	}
	if err := ctx.Err(); err != nil {
		return stepbrep.Mesh{}, err
	}
	return mesh, nil
}

func chordMM(v units.Value) (float64, error) {
	if v.Kind() != units.Length {
		return 0, decad.ErrUnitKind
	}
	value, err := v.In(units.Millimeter)
	if err != nil || math.IsNaN(value) || math.IsInf(value, 0) {
		return 0, decad.ErrNotFinite
	}
	if value <= 0 {
		return 0, decad.ErrDegenerate
	}
	return value, nil
}
