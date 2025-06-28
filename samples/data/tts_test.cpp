#include <atlbase.h>
#include <sapi.h>
#include <windows.h>

int main()
{
	::CoInitialize( nullptr );
	CComPtr<ISpVoice> spVoice;
	HRESULT hr = spVoice.CoCreateInstance( CLSID_SpVoice );
	if ( SUCCEEDED( hr ) )
	{
		spVoice->Speak( L"Hello from SAPI. This is a test polygon.", SPF_DEFAULT, nullptr );
	}
	::CoUninitialize();
	return 0;
}
