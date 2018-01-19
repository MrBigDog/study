#ifndef GWGEOLOGICALUTIL_OPTIONAL_H
#define GWGEOLOGICALUTIL_OPTIONAL_H 1

namespace gwUtil
{
	/**
	 * A template for defining "optional" class members. An optional member has a default value
	 * and a flag indicating whether the member is "set".
	 * This is used extensively in gwUtil's ConfigOptions subsystem.
	 */
	template<typename T> struct optional
	{
		optional() : _set(false), _value(T()), _defaultValue(T()) { }
		optional(T defaultValue) : _set(false), _value(defaultValue), _defaultValue(defaultValue) { }
		optional(T defaultValue, T value) : _set(true), _value(value), _defaultValue(defaultValue) { }
		optional(const optional<T>& rhs) { operator=(rhs); }
		virtual ~optional() { }
		optional<T>& operator =(const optional<T>& rhs) { _set = rhs._set; _value = rhs._value; _defaultValue = rhs._defaultValue; return *this; }
		const T& operator =(const T& value) { _set = true; _value = value; return _value; }
		bool operator ==(const optional<T>& rhs) const { return _set && rhs._set && _value == rhs._value; }
		bool operator !=(const optional<T>& rhs) const { return !((*this) == rhs); }
		bool operator ==(const T& value) const { return _value == value; }
		bool operator !=(const T& value) const { return _value != value; }
		bool operator > (const T& value) const { return _value > value; }
		bool operator >=(const T& value) const { return _value >= value; }
		bool operator < (const T& value) const { return _value < value; }
		bool operator <=(const T& value) const { return _value <= value; }
		bool isSetTo(const T& value) const { return _set && _value == value; } // differs from == in that the value must be explicity set
		bool isSet() const { return _set; }
		void unset() { _set = false; _value = _defaultValue; }
		void clear() { unset(); }

		const T& get() const { return _value; }
		const T& value() const { return _value; }
		const T& defaultValue() const { return _defaultValue; }
		T temp_copy() const { return _value; }

		const T& getOrUse(const T& fallback) const { return _set ? _value : fallback; }

		// gets a mutable reference, automatically setting
		T& mutable_value() { _set = true; return _value; }

		void init(T defValue) { _value = defValue; _defaultValue = defValue; unset(); }

		operator const T*() const { return &_value; }

		T* operator ->() { _set = true; return &_value; }
		const T* operator ->() const { return &_value; }

	private:
		bool _set;
		T _value;
		T _defaultValue;
		typedef T* optional::*unspecified_bool_type;

	public:
		operator unspecified_bool_type() const { return 0; }
	};
}

#endif // GWGEOLOGICALUTIL_OPTIONAL_H
