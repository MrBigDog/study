#ifndef GWGEOLOGICALUTIL_CONTAINERS_H
#define GWGEOLOGICALUTIL_CONTAINERS_H 1

#include "Common.h"
#include "ThreadingUtils.h"
#include <osg/ref_ptr>
#include <osg/observer_ptr>
#include <osg/State>
#include <list>
#include <vector>
#include <set>
#include <map>

namespace gwUtil
{
	template<typename DATA>
	struct fast_set
	{
		typedef DATA                 entry_t;
		typedef std::vector<entry_t> container_t;

		container_t _data;

		typedef typename container_t::iterator       iterator;
		typedef typename container_t::const_iterator const_iterator;

		iterator find(const DATA& value) {
			for (iterator i = _data.begin(); i != _data.end(); ++i) {
				if ((*i) == value) {
					return i;
				}
			}
			return _data.end();
		}

		const_iterator find(const DATA& value) const {
			for (const_iterator i = _data.begin(); i != _data.end(); ++i) {
				if ((*i) == value) {
					return i;
				}
			}
			return _data.end();
		}

		std::pair<iterator, bool> insert(const DATA& value) {
			for (iterator i = _data.begin(); i != _data.end(); ++i) {
				if ((*i) == value) {
					*i = value;
					return std::make_pair(i, false);
				}
			}
			_data.push_back(value);
			iterator n = _data.end(); --n;
			return std::make_pair(n, true);
		}

		const_iterator begin() const { return _data.begin(); }
		const_iterator end() const { return _data.end(); }
		iterator begin() { return _data.begin(); }
		iterator end() { return _data.end(); }
		bool empty() const { return _data.empty(); }
		void clear() { _data.clear(); }
		void erase(iterator i) { _data.erase(i); }
		void erase(const DATA& data) { iterator i = find(data); if (i != _data.end()) _data.erase(i); }
		int size() const { return _data.size(); }
	};

	/**
	 * A std::map-like map that uses a vector and a getUID method for keying.
	 * DATA must have a getUID() method.
	 */
	template<typename KEY, typename DATA>
	struct vector_map
	{
		struct ENTRY {
			inline const KEY&  key()  const { return _key; }
			inline const DATA& data() const { return _data; }
			inline DATA&       data() { return _data; }
			KEY  _key;
			DATA _data;
		};
		typedef std::vector<ENTRY> container_t;

		//typedef std::vector<KEY>  keys_t;
		//typedef std::vector<DATA> container_t;

		typedef typename container_t::iterator       iterator;
		typedef typename container_t::const_iterator const_iterator;

		container_t _container;

		inline DATA& operator[](const KEY& key) {
			for (unsigned i = 0; i < _container.size(); ++i) {
				if (_container[i]._key == key) {
					return _container[i]._data;
				}
			}
			_container.resize(_container.size() + 1);
			_container.back()._key = key;
			return _container.back()._data;
		}

		inline DATA* find(const KEY& key) {
			for (unsigned i = 0; i < _container.size(); ++i) {
				if (_container[i]._key == key) {
					return &_container[i]._data;
				}
			}
			return 0L;
		}

		inline const DATA* find(const KEY& key) const {
			for (unsigned i = 0; i < _container.size(); ++i) {
				if (_container[i]._key == key) {
					return &_container[i]._data;
				}
			}
			return 0L;
		}

		inline const_iterator begin() const { return _container.begin(); }
		inline const_iterator end()   const { return _container.end(); }
		inline iterator begin() { return _container.begin(); }
		inline iterator end() { return _container.end(); }

		inline bool empty() const { return _container.empty(); }

		inline void clear() { _container.clear(); }

		inline void erase(const KEY& key) {
			for (unsigned i = 0; i < _container.size(); ++i) {
				if (_container[i]._key == key) {
					if (i + 1 < _container.size()) {
						_container[i] = _container[_container.size() - 1];
					}
					_container.resize(_container.size() - 1);
					break;
				}
			}
		}

		inline int size() const { return _container.size(); }

		template<typename InputIterator>
		void insert(InputIterator a, InputIterator b) {
			for (InputIterator i = a; i != b; ++i) (*this)[i->first] = i->second;
		}
	};

	/**
	 * A std::map-like container that is faster than std::map for small amounts
	 * of data accessed by a single user
	 */
	template<typename KEY, typename DATA>
	struct fast_map
	{
		typedef std::pair<KEY, DATA>  entry_t;
		typedef std::vector<entry_t> container_t;

		typedef typename container_t::iterator       iterator;
		typedef typename container_t::const_iterator const_iterator;

		container_t _data;
		KEY         _lastKey;

		DATA& operator[] (const KEY& key) {
			for (iterator i = _data.begin(); i != _data.end(); ++i) {
				if (i->first == key) {
					return i->second;
				}
			}
			_data.push_back(entry_t(key, DATA()));
			return _data.back().second;
		}

		iterator find(const KEY& key) {
			for (iterator i = _data.begin(); i != _data.end(); ++i) {
				if (i->first == key) {
					return i;
				}
			}
			return end();
		}

		const_iterator find(const KEY& key) const {
			for (const_iterator i = _data.begin(); i != _data.end(); ++i) {
				if (i->first == key) {
					return i;
				}
			}
			return end();
		}

		const_iterator begin() const { return _data.begin(); }
		const_iterator end() const { return _data.end(); }
		iterator begin() { return _data.begin(); }
		iterator end() { return _data.end(); }

		bool empty() const { return _data.empty(); }

		void clear() {
			_data.clear();
		}

		iterator erase(iterator i) {
			return _data.erase(i);
		}

		iterator erase(iterator i0, iterator i1) {
			return _data.erase(i0, i1);
		}

		void erase(const KEY& key) {
			iterator i = find(key);
			if (i != end()) {
				erase(i);
			}
		}

		int size() const { return _data.size(); }

		template<typename InputIterator>
		void insert(InputIterator a, InputIterator b) {
			for (InputIterator i = a; i != b; ++i) (*this)[i->first] = i->second;
		}
	};

	//------------------------------------------------------------------------

	struct CacheStats
	{
	public:
		CacheStats(unsigned entries, unsigned maxEntries, unsigned queries, float hitRatio)
			: _entries(entries), _maxEntries(maxEntries), _queries(queries), _hitRatio(hitRatio) { }

		/** dtor */
		virtual ~CacheStats() { }

		unsigned _entries;
		unsigned _maxEntries;
		unsigned _queries;
		float    _hitRatio;
	};

	//------------------------------------------------------------------------

	/**
	 * Least-recently-used cache class.
	 * K = key type, T = value type
	 *
	 * usage:
	 *    LRUCache<K,T> cache;
	 *    cache.put( key, value );
	 *    LRUCache.Record rec = cache.get( key );
	 *    if ( rec.valid() )
	 *        const T& value = rec.value();
	 */
	template<typename K, typename T, typename COMPARE = std::less<K> >
	class LRUCache
	{
	public:
		struct Record {
			Record() : _valid(false) { }
			Record(const T& value) : _value(value), _valid(true) { }
			bool valid() const { return _valid; }
			const T& value() const { return _value; }
		private:
			bool _valid;
			T    _value;
			friend class LRUCache;
		};

		struct Functor {
			virtual void operator()(const K& key, const T& value) = 0;
		};

	protected:
		typedef typename std::list<K>::iterator      lru_iter;
		typedef typename std::list<K>                lru_type;
		typedef typename std::pair<T, lru_iter>      map_value_type;
		typedef typename std::map<K, map_value_type> map_type;
		typedef typename map_type::iterator          map_iter;
		typedef typename map_type::const_iterator    map_const_iter;

		map_type _map;
		lru_type _lru;
		unsigned _max;
		unsigned _buf;
		unsigned _queries;
		unsigned _hits;
		bool     _threadsafe;
		mutable Threading::Mutex _mutex;

	public:
		LRUCache(unsigned max = 100) : _max(max), _threadsafe(false) {
			_buf = _max / 10;
			_queries = 0;
			_hits = 0;
		}
		LRUCache(bool threadsafe, unsigned max = 100) : _max(max), _threadsafe(threadsafe) {
			_buf = _max / 10;
			_queries = 0;
			_hits = 0;
		}

		/** dtor */
		virtual ~LRUCache() { }

		void insert(const K& key, const T& value) {
			if (_threadsafe) {
				Threading::ScopedMutexLock lock(_mutex);
				insert_impl(key, value);
			}
			else {
				insert_impl(key, value);
			}
		}

		bool get(const K& key, Record& out) {
			if (_threadsafe) {
				Threading::ScopedMutexLock lock(_mutex);
				get_impl(key, out);
			}
			else {
				get_impl(key, out);
			}
			return out.valid();
		}

		bool has(const K& key) {
			if (_threadsafe) {
				Threading::ScopedMutexLock lock(_mutex);
				return has_impl(key);
			}
			else {
				return has_impl(key);
			}
		}

		void erase(const K& key) {
			if (_threadsafe) {
				Threading::ScopedMutexLock lock(_mutex);
				erase_impl(key);
			}
			else {
				erase_impl(key);
			}
		}

		void clear() {
			if (_threadsafe) {
				Threading::ScopedMutexLock lock(_mutex);
				clear_impl();
			}
			else {
				clear_impl();
			}
		}

		void setMaxSize(unsigned max) {
			if (_threadsafe) {
				Threading::ScopedMutexLock lock(_mutex);
				setMaxSize_impl(max);
			}
			else {
				setMaxSize_impl(max);
			}
		}

		unsigned getMaxSize() const {
			return _max;
		}

		CacheStats getStats() const {
			return CacheStats(
				_map.size(), _max, _queries, _queries > 0 ? (float)_hits / (float)_queries : 0.0f);
		}

		void iterate(Functor& functor) const {
			if (_threadsafe) {
				Threading::ScopedMutexLock lock(_mutex);
				iterate_impl(functor);
			}
			else {
				iterate_impl(functor);
			}
		}

	private:

		void insert_impl(const K& key, const T& value) {
			map_iter mi = _map.find(key);
			if (mi != _map.end()) {
				_lru.erase(mi->second.second);
				mi->second.first = value;
				_lru.push_back(key);
				mi->second.second = _lru.end();
				mi->second.second--;
			}
			else {
				_lru.push_back(key);
				lru_iter last = _lru.end(); last--;
				_map[key] = std::make_pair(value, last);
			}

			if (_map.size() > _max) {
				for (unsigned i = 0; i < _buf; ++i) {
					const K& key = _lru.front();
					_map.erase(key);
					_lru.pop_front();
				}
			}
		}

		void get_impl(const K& key, Record& result) {
			_queries++;
			map_iter mi = _map.find(key);
			if (mi != _map.end()) {
				_lru.erase(mi->second.second);
				_lru.push_back(key);
				lru_iter new_iter = _lru.end(); new_iter--;
				mi->second.second = new_iter;
				_hits++;
				result._value = mi->second.first;
				result._valid = true;
			}
		}

		bool has_impl(const K& key) {
			return _map.find(key) != _map.end();
		}

		void erase_impl(const K& key) {
			map_iter mi = _map.find(key);
			if (mi != _map.end()) {
				_lru.erase(mi->second.second);
				_map.erase(mi);
			}
		}

		void clear_impl() {
			_lru.clear();
			_map.clear();
			_queries = 0;
			_hits = 0;
		}

		void setMaxSize_impl(unsigned max) {
			_max = max;
			_buf = max / 10;
			while (_map.size() > _max) {
				const K& key = _lru.front();
				_map.erase(key);
				_lru.pop_front();
			}
		}

		void iterate_impl(Functor& f) const {
			for (map_const_iter i = _map.begin(); i != _map.end(); ++i) {
				f(i->first, i->second.first);
			}
		}
	};

	//--------------------------------------------------------------------

	/**
	 * Same of osg::MixinVector, but with a superclass template parameter.
	 */
	template<class ValueT, class SuperClass>
	class MixinVector : public SuperClass
	{
		typedef typename std::vector<ValueT> vector_type;
	public:
		typedef typename vector_type::allocator_type allocator_type;
		typedef typename vector_type::value_type value_type;
		typedef typename vector_type::const_pointer const_pointer;
		typedef typename vector_type::pointer pointer;
		typedef typename vector_type::const_reference const_reference;
		typedef typename vector_type::reference reference;
		typedef typename vector_type::const_iterator const_iterator;
		typedef typename vector_type::iterator iterator;
		typedef typename vector_type::const_reverse_iterator const_reverse_iterator;
		typedef typename vector_type::reverse_iterator reverse_iterator;
		typedef typename vector_type::size_type size_type;
		typedef typename vector_type::difference_type difference_type;

		explicit MixinVector() : _impl()
		{
		}

		explicit MixinVector(size_type initial_size, const value_type& fill_value = value_type())
			: _impl(initial_size, fill_value)
		{
		}

		template<class InputIterator>
		MixinVector(InputIterator first, InputIterator last)
			: _impl(first, last)
		{
		}

		MixinVector(const vector_type& other)
			: _impl(other)
		{
		}

		MixinVector(const MixinVector& other)
			: _impl(other._impl)
		{
		}

		MixinVector& operator=(const vector_type& other)
		{
			_impl = other;
			return *this;
		}

		MixinVector& operator=(const MixinVector& other)
		{
			_impl = other._impl;
			return *this;
		}

		virtual ~MixinVector() {}

		void clear() { _impl.clear(); }
		void resize(size_type new_size, const value_type& fill_value = value_type()) { _impl.resize(new_size, fill_value); }
		void reserve(size_type new_capacity) { _impl.reserve(new_capacity); }

		void swap(vector_type& other) { _impl.swap(other); }
		void swap(MixinVector& other) { _impl.swap(other._impl); }

		bool empty() const { return _impl.empty(); }
		size_type size() const { return _impl.size(); }
		size_type capacity() const { return _impl.capacity(); }
		size_type max_size() const { return _impl.max_size(); }
		allocator_type get_allocator() const { return _impl.get_allocator(); }

		const_iterator begin() const { return _impl.begin(); }
		iterator begin() { return _impl.begin(); }
		const_iterator end() const { return _impl.end(); }
		iterator end() { return _impl.end(); }

		const_reverse_iterator rbegin() const { return _impl.rbegin(); }
		reverse_iterator rbegin() { return _impl.rbegin(); }
		const_reverse_iterator rend() const { return _impl.rend(); }
		reverse_iterator rend() { return _impl.rend(); }

		const_reference operator[](size_type index) const { return _impl[index]; }
		reference operator[](size_type index) { return _impl[index]; }

		const_reference at(size_type index) const { return _impl.at(index); }
		reference at(size_type index) { return _impl.at(index); }

		void assign(size_type count, const value_type& value) { _impl.assign(count, value); }
		template<class Iter>
		void assign(Iter first, Iter last) { _impl.assign(first, last); }

		void push_back(const value_type& value) { _impl.push_back(value); }
		void pop_back() { _impl.pop_back(); }

		iterator erase(iterator where) { return _impl.erase(where); }
		iterator erase(iterator first, iterator last) { return _impl.erase(first, last); }

		iterator insert(iterator where, const value_type& value) { return _impl.insert(where, value); }

		template<class InputIterator>
		void insert(iterator where, InputIterator first, InputIterator last)
		{
			_impl.insert(where, first, last);
		}

		void insert(iterator where, size_type count, const value_type& value)
		{
			_impl.insert(where, count, value);
		}

		const_reference back() const { return _impl.back(); }
		reference back() { return _impl.back(); }
		const_reference front() const { return _impl.front(); }
		reference front() { return _impl.front(); }

		vector_type& asVector() { return _impl; }
		const vector_type& asVector() const { return _impl; }

		friend inline bool operator==(const MixinVector<ValueT, SuperClass>& left, const MixinVector<ValueT, SuperClass>& right) { return left._impl == right._impl; }
		friend inline bool operator==(const MixinVector<ValueT, SuperClass>& left, const std::vector<ValueT>& right) { return left._impl == right; }
		friend inline bool operator==(const std::vector<ValueT>& left, const MixinVector<ValueT, SuperClass>& right) { return left == right._impl; }

		friend inline bool operator!=(const MixinVector<ValueT, SuperClass>& left, const MixinVector<ValueT, SuperClass>& right) { return left._impl != right._impl; }
		friend inline bool operator!=(const MixinVector<ValueT, SuperClass>& left, const std::vector<ValueT>& right) { return left._impl != right; }
		friend inline bool operator!=(const std::vector<ValueT>& left, const MixinVector<ValueT, SuperClass>& right) { return left != right._impl; }

		friend inline bool operator<(const MixinVector<ValueT, SuperClass>& left, const MixinVector<ValueT, SuperClass>& right) { return left._impl < right._impl; }
		friend inline bool operator<(const MixinVector<ValueT, SuperClass>& left, const std::vector<ValueT>& right) { return left._impl < right; }
		friend inline bool operator<(const std::vector<ValueT>& left, const MixinVector<ValueT, SuperClass>& right) { return left < right._impl; }

		friend inline bool operator>(const MixinVector<ValueT, SuperClass>& left, const MixinVector<ValueT, SuperClass>& right) { return left._impl > right._impl; }
		friend inline bool operator>(const MixinVector<ValueT, SuperClass>& left, const std::vector<ValueT>& right) { return left._impl > right; }
		friend inline bool operator>(const std::vector<ValueT>& left, const MixinVector<ValueT, SuperClass>& right) { return left > right._impl; }

		friend inline bool operator<=(const MixinVector<ValueT, SuperClass>& left, const MixinVector<ValueT, SuperClass>& right) { return left._impl <= right._impl; }
		friend inline bool operator<=(const MixinVector<ValueT, SuperClass>& left, const std::vector<ValueT>& right) { return left._impl <= right; }
		friend inline bool operator<=(const std::vector<ValueT>& left, const MixinVector<ValueT, SuperClass>& right) { return left <= right._impl; }

		friend inline bool operator>=(const MixinVector<ValueT, SuperClass>& left, const MixinVector<ValueT, SuperClass>& right) { return left._impl >= right._impl; }
		friend inline bool operator>=(const MixinVector<ValueT, SuperClass>& left, const std::vector<ValueT>& right) { return left._impl >= right; }
		friend inline bool operator>=(const std::vector<ValueT>& left, const MixinVector<ValueT, SuperClass>& right) { return left >= right._impl; }

	private:
		vector_type _impl;
	};


	/** Template for per-thread data storage */
	template<typename T>
	struct PerThread
	{
		T& get() {
			Threading::ScopedMutexLock lock(_mutex);
			return _data[Threading::getCurrentThreadId()];
		}
	private:
		std::map<unsigned, T> _data;
		Threading::Mutex     _mutex;
	};


	/** Template for thread safe per-object data storage */
	template<typename KEY, typename DATA>
	struct PerObjectMap
	{
		DATA& get(KEY k)
		{
			{
				gwUtil::Threading::ScopedReadLock readLock(_mutex);
				typename std::map<KEY, DATA>::iterator i = _data.find(k);
				if (i != _data.end())
					return i->second;
			}
			{
				gwUtil::Threading::ScopedWriteLock lock(_mutex);
				typename std::map<KEY, DATA>::iterator i = _data.find(k);
				if (i != _data.end())
					return i->second;
				else
					return _data[k];
			}
		}

		void remove(KEY k)
		{
			gwUtil::Threading::ScopedWriteLock exclusive(_mutex);
			_data.erase(k);
		}

	private:
		std::map<KEY, DATA>                  _data;
		gwUtil::Threading::ReadWriteMutex _mutex;
	};

	/** Template for thread safe per-object data storage */
	template<typename KEY, typename DATA>
	struct PerObjectFastMap
	{
		DATA& get(KEY k)
		{
			{
				gwUtil::Threading::ScopedReadLock readLock(_mutex);
				typename gwUtil::fast_map<KEY, DATA>::iterator i = _data.find(k);
				if (i != _data.end())
					return i->second;
			}
			{
				gwUtil::Threading::ScopedWriteLock lock(_mutex);
				typename gwUtil::fast_map<KEY, DATA>::iterator i = _data.find(k);
				if (i != _data.end())
					return i->second;
				else
					return _data[k];
			}
		}

		void remove(KEY k)
		{
			gwUtil::Threading::ScopedWriteLock exclusive(_mutex);
			_data.erase(k);
		}

	private:
		gwUtil::fast_map<KEY, DATA>        _data;
		gwUtil::Threading::ReadWriteMutex _mutex;
	};

	/** Template for thread safe per-object data storage */
	template<typename KEY, typename DATA>
	struct PerObjectRefMap
	{
		DATA* get(KEY k)
		{
			gwUtil::Threading::ScopedReadLock lock(_mutex);
			typename std::map<KEY, osg::ref_ptr<DATA > >::const_iterator i = _data.find(k);
			if (i != _data.end())
				return i->second.get();

			return 0L;
		}

		DATA* getOrCreate(KEY k, DATA* newDataIfNeeded)
		{
			osg::ref_ptr<DATA> _refReleaser = newDataIfNeeded;
			{
				gwUtil::Threading::ScopedReadLock lock(_mutex);
				typename std::map<KEY, osg::ref_ptr<DATA> >::const_iterator i = _data.find(k);
				if (i != _data.end())
					return i->second.get();
			}

			{
				gwUtil::Threading::ScopedWriteLock lock(_mutex);
				typename std::map<KEY, osg::ref_ptr<DATA> >::iterator i = _data.find(k);
				if (i != _data.end())
					return i->second.get();

				_data[k] = newDataIfNeeded;
				return newDataIfNeeded;
			}
		}

		void remove(KEY k)
		{
			gwUtil::Threading::ScopedWriteLock exclusive(_mutex);
			_data.erase(k);
		}

		void remove(DATA* data)
		{
			gwUtil::Threading::ScopedWriteLock exclusive(_mutex);
			for (typename std::map<KEY, osg::ref_ptr<DATA> >::iterator i = _data.begin(); i != _data.end(); ++i)
			{
				if (i->second.get() == data)
				{
					_data.erase(i);
					break;
				}
			}
		}

	private:
		std::map<KEY, osg::ref_ptr<DATA> >    _data;
		gwUtil::Threading::ReadWriteMutex  _mutex;
	};

	/** Template for thread safe per-object data storage */
	template<typename KEY, typename DATA>
	struct PerObjectObsMap
	{
		DATA* get(KEY k)
		{
			gwUtil::Threading::ScopedReadLock lock(_mutex);
			typename std::map<KEY, osg::observer_ptr<DATA> >::const_iterator i = _data.find(k);
			if (i != _data.end())
				return i->second.get();

			return 0L;
		}

		DATA* getOrCreate(KEY k, DATA* newDataIfNeeded)
		{
			osg::ref_ptr<DATA> _refReleaser = newDataIfNeeded;
			{
				gwUtil::Threading::ScopedReadLock lock(_mutex);
				typename std::map<KEY, osg::observer_ptr<DATA> >::const_iterator i = _data.find(k);
				if (i != _data.end())
					return i->second.get();
			}

			{
				gwUtil::Threading::ScopedWriteLock lock(_mutex);
				typename std::map<KEY, osg::observer_ptr<DATA> >::iterator i = _data.find(k);
				if (i != _data.end())
					return i->second.get();

				_data[k] = newDataIfNeeded;
				return newDataIfNeeded;
			}
		}

		void remove(KEY k)
		{
			gwUtil::Threading::ScopedWriteLock exclusive(_mutex);
			_data.erase(k);
		}

		void remove(DATA* data)
		{
			gwUtil::Threading::ScopedWriteLock exclusive(_mutex);
			for (typename std::map<KEY, osg::observer_ptr<DATA> >::iterator i = _data.begin(); i != _data.end(); ++i)
			{
				if (i->second.get() == data)
				{
					_data.erase(i);
					break;
				}
			}
		}

	private:
		std::map<KEY, osg::observer_ptr<DATA> >    _data;
		gwUtil::Threading::ReadWriteMutex       _mutex;
	};


	/** Template for thread safe observer set */
	template<typename T>
	struct ThreadSafeObserverSet
	{
		typedef void(*Functor)(T*);
		typedef void(*ConstFunctor)(const T*);

		void iterate(const Functor& f)
		{
			gwUtil::Threading::ScopedWriteLock lock(_mutex);
			for (typename std::set<T>::iterator i = _data.begin(); i != _data.end(); )
			{
				if (i->valid())
				{
					f(i->get());
					++i;
				}
				else
				{
					i = _data.erase(i);
				}
			}
		}

		void iterate(const ConstFunctor& f)
		{
			gwUtil::Threading::ScopedReadLock lock(_mutex);
			for (typename std::set<T>::iterator i = _data.begin(); i != _data.end(); )
			{
				if (i->valid())
				{
					f(i->get());
					++i;
				}
				else
				{
					i = _data.erase(i);
				}
			}
		}

		void insert(T* obj)
		{
			gwUtil::Threading::ScopedWriteLock lock(_mutex);
			_data.insert(obj);
		}

		void remove(T* obj)
		{
			gwUtil::Threading::ScopedWriteLock lock(_mutex);
			_data.erase(obj);
		}

	private:
		std::set<osg::observer_ptr<T> >      _data;
		gwUtil::Threading::ReadWriteMutex  _mutex;
	};
}

#endif // OSGEARTH_CONTAINERS_H
