
#include <osgEarth/Common>
#include <osgEarth/IOTypes>
#include <osg/ref_ptr>
#include <osg/Referenced>
#include <osgDB/ReaderWriter>
#include <sstream>
#include <iostream>
#include <string>
#include <map>
#include <vector>

namespace osgEarth
{
    class ProgressCallback;

    /**
     * Proxy server configuration.
     */
    class OSGEARTH_EXPORT ProxySettings
    {
    public:
        ProxySettings( const Config& conf =Config() );
        ProxySettings( const std::string& host, int port );

        virtual ~ProxySettings() { }

        std::string& hostName() { return _hostName; }
        const std::string& hostName() const { return _hostName; }

        int& port() { return _port; }
        const int& port() const { return _port; }

        std::string& userName() { return _userName; }
        const std::string& userName() const { return _userName; }

        std::string& password() { return _password; }
        const std::string& password() const { return _password; }

        void apply(osgDB::Options* dbOptions) const;
        static bool fromOptions( const osgDB::Options* dbOptions, optional<ProxySettings>& out );

    public:
        virtual Config getConfig() const;
        virtual void mergeConfig( const Config& conf );

    protected:
        std::string _hostName;
        int _port;
        std::string _userName;
        std::string _password;
    };

    typedef std::map<std::string,std::string> Headers;


    /**
     * An HTTP request for use with the HTTPClient class.
     */
    class OSGEARTH_EXPORT HTTPRequest
    {
    public:
        /** Constructs a new HTTP request that will acces the specified base URL. */
        HTTPRequest( const std::string& url );

        /** copy constructor. */
        HTTPRequest( const HTTPRequest& rhs );

        /** dtor */
        virtual ~HTTPRequest() { }

        /** Adds an HTTP parameter to the request query string. */
        void addParameter( const std::string& name, const std::string& value );
        void addParameter( const std::string& name, int value );
        void addParameter( const std::string& name, double value );        
        
        typedef std::map<std::string,std::string> Parameters;

        /** Ready-only access to the parameter list (as built with addParameter) */
        const Parameters& getParameters() const;        

        void addHeader( const std::string& name, const std::string& value );

        const Headers& getHeaders() const;

        /**
         * Sets the last modified date of any locally cached data for this request.  This will 
         * automatically add a If-Modified-Since header to the request
         */
        void setLastModified( const DateTime &lastModified );

        /** Gets a copy of the complete URL (base URL + query string) for this request */
        std::string getURL() const;
        
    private:
        Parameters _parameters;
        Headers _headers;
        std::string _url;
    };

    /**
     * An HTTP response object for use with the HTTPClient class - supports
     * multi-part mime responses.
     */
    class OSGEARTH_EXPORT HTTPResponse
    {
    public:
        enum Code {
            NONE         = 0,
            OK           = 200,
            NOT_MODIFIED = 304,
            BAD_REQUEST  = 400,
            NOT_FOUND    = 404,
            CONFLICT     = 409,
            SERVER_ERROR = 500
        };

    public:
        /** Constructs a response with the specified HTTP response code */
        HTTPResponse( long code =0L );

        /** Copy constructor */
        HTTPResponse( const HTTPResponse& rhs );

        /** dtor */
        virtual ~HTTPResponse() { }

        /** Gets the HTTP response code (Code) in this response */
        unsigned getCode() const;

        /** True is the HTTP response code is OK (200) */
        bool isOK() const;

        /** True if the request associated with this response was cancelled before it completed */
        bool isCancelled() const;

        /** Gets the number of parts in a (possibly multipart mime) response */
        unsigned int getNumParts() const;

        /** Gets the input stream for the nth part in the response */
        std::istream& getPartStream( unsigned int n ) const;

        /** Gets the nth response part as a string */
        std::string getPartAsString( unsigned int n ) const;

        /** Gets the length of the nth response part */
        unsigned int getPartSize( unsigned int n ) const;
        
        /** Gets the HTTP header associated with the nth multipart/mime response part */
        const std::string& getPartHeader( unsigned int n, const std::string& name ) const;

        /** Gets the master mime-type returned by the request */
        const std::string& getMimeType() const;

        /** How long did it take to fetch this response (in seconds) */
        double getDuration() const { return _duration_s; }        

    private:
        struct Part : public osg::Referenced
        {
            Part() : _size(0) { }            
            Headers _headers;
            unsigned int _size;
            std::stringstream _stream;
        };
        typedef std::vector< osg::ref_ptr<Part> > Parts;
        Parts       _parts;
        long        _response_code;
        std::string _mimeType;
        bool        _cancelled;
        double      _duration_s;
        TimeStamp   _lastModified;

        Config getHeadersAsConfig() const;

        friend class HTTPClient;
    };

    /**
     * Object that lets you modify and incoming URL before it's passed to the server
     */
    struct OSGEARTH_EXPORT URLRewriter : public osg::Referenced
    {    
        virtual std::string rewrite( const std::string& url ) = 0;
    };

	/**
	 *
	 * A CURL configuration handler to apply CURL settings. It can be used for setting client certificates
	 */
	struct OSGEARTH_EXPORT CurlConfigHandler : public osg::Referenced
	{
		virtual void onInitialize(void* curl_handle) = 0;
		virtual void onGet(void* curl_handle) = 0;
	};
	
	/**
     * Utility class for making HTTP requests.
     *
     * TODO: This class will actually read data from disk as well, and therefore should
     * probably be renamed. It analyzes the URI and decides whether to make an  HTTP request
     * or to read from disk.
     */
    class OSGEARTH_EXPORT HTTPClient
    {
    public:
        /**
         * Returns true is the result code represents a recoverable situation,
         * i.e. one in which retrying might work.
         */
        static bool isRecoverable( ReadResult::Code code )
        {
            return
                code == ReadResult::RESULT_OK ||                
                code == ReadResult::RESULT_SERVER_ERROR ||
                code == ReadResult::RESULT_TIMEOUT ||
                code == ReadResult::RESULT_CANCELED;
        }

        /** Gest the user-agent string that all HTTP requests will use.
            TODO: This should probably move into the Registry */
        static const std::string& getUserAgent();

        /** Sets a user-agent string to use in all HTTP requests.
            TODO: This should probably move into the Registry */
        static void setUserAgent(const std::string& userAgent);

        /** Sets up proxy info to use in all HTTP requests.
            TODO: This should probably move into the Registry */
        static void setProxySettings( const ProxySettings &proxySettings );

        /**
           Gets the timeout in seconds to use for HTTP requests.*/
        static long getTimeout();

        /**
           Sets the timeout in seconds to use for HTTP requests.
           Setting to 0 (default) is infinite timeout */
        static void setTimeout( long timeout );

        /**
           Gets the timeout in seconds to use for HTTP connect requests.*/
        static long getConnectTimeout();

        /**
           Sets the timeout in seconds to use for HTTP connect requests.
           Setting to 0 (default) is infinite timeout */
        static void setConnectTimeout( long timeout );

        /**
         * Gets the URLRewriter that is used to modify urls before sending them to the server
         */
        static URLRewriter* getURLRewriter();

        /**
         * Sets the URLRewriter that is used to modify urls before sending them to the server         
         */
        static void setURLRewriter( URLRewriter* rewriter );

		static CurlConfigHandler* getCurlConfigHandler();

		/**
		* Sets the CurlConfigHandler to configurate the CURL library. It can be used for apply client certificates
		*/
		static void setCurlConfighandler(CurlConfigHandler* handler);
		
		/**
         * One time thread safe initialization. In osgEarth, you don't need
         * to call this directly; osgEarth::Registry will call it at
         * startup.
         */
        static void globalInit();


    public:
        /**
         * Reads an image.
         */
        static ReadResult readImage(
            const HTTPRequest&    request,
            const osgDB::Options* dbOptions =0L,
            ProgressCallback*     progress  =0L );

        /**
         * Reads an osg::Node.
         */
        static ReadResult readNode(
            const HTTPRequest&    request,
            const osgDB::Options* dbOptions =0L,
            ProgressCallback*     progress  =0L );

        /**
         * Reads an object.
         */
        static ReadResult readObject(
            const HTTPRequest&    request,
            const osgDB::Options* dbOptions =0L,
            ProgressCallback*     progress  =0L );

        /**
         * Reads a string.
         */
        static ReadResult readString(
            const HTTPRequest&    request,
            const osgDB::Options* dbOptions =0L,
            ProgressCallback*     progress  =0L );

        /**
         * Downloads a file directly to disk.
         */
        static bool download(
            const std::string& uri,
            const std::string& localPath );

    public:

        /**
         * Performs an HTTP "GET".
         */
        static HTTPResponse get( const HTTPRequest&    request,
                                 const osgDB::Options* dbOptions =0L,
                                 ProgressCallback*     progress  =0L );

        static HTTPResponse get( const std::string&    url,
                                 const osgDB::Options* options  =0L,
                                 ProgressCallback*     progress =0L );

    public:
        HTTPClient();
        virtual ~HTTPClient();

    private:

        void readOptions( const osgDB::ReaderWriter::Options* options, std::string &proxy_host, std::string &proxy_port ) const;

        HTTPResponse doGet( const HTTPRequest&    request,
                            const osgDB::Options* options  =0L,
                            ProgressCallback*     callback =0L ) const;
        
        ReadResult doReadObject(
            const HTTPRequest&    request,
            const osgDB::Options* dbOptions,
            ProgressCallback*     progress );

        ReadResult doReadImage(
            const HTTPRequest&    request,
            const osgDB::Options* dbOptions,
            ProgressCallback*     progress );

        ReadResult doReadNode(
            const HTTPRequest&    request,
            const osgDB::Options* dbOptions,
            ProgressCallback*     progress );

        ReadResult doReadString(
            const HTTPRequest&    request,
            const osgDB::Options* dbOptions,
            ProgressCallback*     progress );

        /**
         * Convenience method for downloading a URL directly to a file
         */
        bool doDownload(const std::string& url, const std::string& filename);

    private:
        void*       _curl_handle;
        std::string _previousPassword;
        long        _previousHttpAuthentication;
        bool        _initialized;
        long        _simResponseCode;

        void initialize() const;
        void initializeImpl();


        static HTTPClient& getClient();

    private:
        bool decodeMultipartStream(
            const std::string&   boundary,
            HTTPResponse::Part*  input,
            HTTPResponse::Parts& output) const;
    };
}

#endif // OSGEARTH_HTTP_CLIENT_H
