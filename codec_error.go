package decad

import (
	"bytes"
	"encoding"
	"encoding/json"
	"errors"
	"fmt"
	"reflect"
	"strings"
)

// codecPathError carries a path relative to the value currently being
// decoded. Nested codecs prepend their field or array position; Recipe's root
// decoder turns the result into one public RecipeError.
type codecPathError struct {
	path string
	err  error
}

func (e *codecPathError) Error() string {
	if e.path == "" {
		return e.err.Error()
	}
	return fmt.Sprintf(`%s: %s`, e.path, e.err)
}

func (e *codecPathError) Unwrap() error { return e.err }

func joinCodecPath(prefix, suffix string) string {
	switch {
	case prefix == "":
		return suffix
	case suffix == "":
		return prefix
	case strings.HasPrefix(suffix, "["):
		return prefix + suffix
	default:
		return prefix + "." + suffix
	}
}

func prependCodecPath(err error, prefix string) error {
	if err == nil {
		return nil
	}
	var pathErr *codecPathError
	if errors.As(err, &pathErr) {
		return &codecPathError{
			path: joinCodecPath(prefix, pathErr.path),
			err:  pathErr.err,
		}
	}
	return &codecPathError{path: prefix, err: err}
}

// codecJSONError retains the field path encoding/json already found for a
// primitive type error. Manually decoded arrays add their own indices.
func codecJSONError(err error) error {
	if err == nil {
		return nil
	}
	var typeErr *json.UnmarshalTypeError
	if errors.As(err, &typeErr) && typeErr.Field != "" {
		return prependCodecPath(err, typeErr.Field)
	}
	return err
}

// codecJSONErrorAt fills the gap left by encoding/json for errors returned by
// a field's custom decoder. Those errors do not carry the containing field,
// and array type errors do not carry the element index. The original decode
// remains authoritative; this second, read-only walk follows its error
// precedence only to find the returned error's location.
func codecJSONErrorAt(data []byte, dst any, err error) error {
	if err == nil {
		return nil
	}
	var pathErr *codecPathError
	hasPathErr := errors.As(err, &pathErr)
	path := codecFailurePath(data, reflect.TypeOf(dst))
	if path != "" {
		if hasPathErr {
			return prependCodecPath(pathErr, path)
		}
		return prependCodecPath(err, path)
	}
	if hasPathErr {
		return err
	}
	return codecJSONError(err)
}

var (
	jsonUnmarshalerType = reflect.TypeFor[json.Unmarshaler]()
	textUnmarshalerType = reflect.TypeFor[encoding.TextUnmarshaler]()
)

func codecFailurePath(data []byte, typ reflect.Type) string {
	failure := codecFailurePathInfo(data, typ)
	return failure.path
}

type codecFailureInfo struct {
	path  string
	fatal bool
}

func codecFailurePathInfo(data []byte, typ reflect.Type) codecFailureInfo {
	if typ == nil {
		return codecFailureInfo{}
	}
	for typ.Kind() == reflect.Pointer {
		typ = typ.Elem()
	}
	if codecCustomDecoderRuns(data, typ) {
		return codecFailureInfo{fatal: true}
	}

	switch typ.Kind() {
	case reflect.Struct:
		fields, ok := codecObjectFields(data)
		if !ok {
			return codecFailureInfo{}
		}
		var firstDeferred codecFailureInfo
		for _, wireField := range fields {
			field, ok := codecStructField(typ, wireField.name)
			if !ok {
				continue
			}
			candidate := reflect.New(field.Type).Interface()
			if err := json.Unmarshal(wireField.data, candidate); err != nil {
				nested := codecFailurePathInfo(wireField.data, field.Type)
				failure := codecFailureInfo{
					path: joinCodecPath(wireField.name, nested.path),
				}
				if nested.fatal || !isDeferredCodecError(err) {
					failure.fatal = true
					return failure
				}
				if firstDeferred.path == "" {
					firstDeferred = failure
				}
			}
		}
		return firstDeferred
	case reflect.Slice, reflect.Array:
		var raw []json.RawMessage
		if err := json.Unmarshal(data, &raw); err != nil {
			return codecFailureInfo{}
		}
		var firstDeferred codecFailureInfo
		for i, elementData := range raw {
			candidate := reflect.New(typ.Elem()).Interface()
			if err := json.Unmarshal(elementData, candidate); err != nil {
				nested := codecFailurePathInfo(elementData, typ.Elem())
				failure := codecFailureInfo{
					path: joinCodecPath(fmt.Sprintf(`[%d]`, i), nested.path),
				}
				if nested.fatal || !isDeferredCodecError(err) {
					failure.fatal = true
					return failure
				}
				if firstDeferred.path == "" {
					firstDeferred = failure
				}
			}
		}
		return firstDeferred
	}
	return codecFailureInfo{}
}

func codecCustomDecoderRuns(data []byte, typ reflect.Type) bool {
	for typ.Kind() == reflect.Pointer {
		typ = typ.Elem()
	}
	if typ.Implements(jsonUnmarshalerType) || reflect.PointerTo(typ).Implements(jsonUnmarshalerType) {
		return true
	}
	if !typ.Implements(textUnmarshalerType) && !reflect.PointerTo(typ).Implements(textUnmarshalerType) {
		return false
	}
	data = bytes.TrimSpace(data)
	return len(data) > 0 && data[0] == '"'
}

func isDeferredCodecError(err error) bool {
	var typeErr *json.UnmarshalTypeError
	return errors.As(err, &typeErr)
}

type codecRawField struct {
	name string
	data json.RawMessage
}

func codecObjectFields(data []byte) ([]codecRawField, bool) {
	decoder := json.NewDecoder(bytes.NewReader(data))
	start, err := decoder.Token()
	if err != nil || start != json.Delim('{') {
		return nil, false
	}
	fields := make([]codecRawField, 0)
	for decoder.More() {
		token, err := decoder.Token()
		if err != nil {
			return nil, false
		}
		name, ok := token.(string)
		if !ok {
			return nil, false
		}
		var fieldData json.RawMessage
		if err := decoder.Decode(&fieldData); err != nil {
			return nil, false
		}
		fields = append(fields, codecRawField{name: name, data: fieldData})
	}
	if _, err := decoder.Token(); err != nil {
		return nil, false
	}
	return fields, true
}

func codecStructField(typ reflect.Type, wireName string) (reflect.StructField, bool) {
	var folded reflect.StructField
	for i := range typ.NumField() {
		field := typ.Field(i)
		if !field.IsExported() {
			continue
		}
		name := field.Name
		if tag, ok := field.Tag.Lookup("json"); ok {
			name = strings.Split(tag, ",")[0]
			if name == "-" {
				continue
			}
			if name == "" {
				name = field.Name
			}
		}
		if wireName == name {
			return field, true
		}
		if folded.Name == "" && strings.EqualFold(wireName, name) {
			folded = field
		}
	}
	return folded, folded.Name != ""
}

func newRecipeDecodeError(step int, prefix string, err error) error {
	path := prefix
	cause := err
	var pathErr *codecPathError
	if errors.As(err, &pathErr) {
		path = joinCodecPath(prefix, pathErr.path)
		cause = pathErr.err
	}
	return &RecipeError{
		StepIndex: step,
		Path:      path,
		Kind:      ErrInvalidRecipe,
		Err:       cause,
	}
}
