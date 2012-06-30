#pragma once
#ifndef PS_STRING_H
#define PS_STRING_H

#include <istream>
#include <vector>
#include "PS_StringBase.h"

namespace PS{

	//AnsiString a replacement for std and char* strings
	class CAString : public CTString<char>
	{
	public:	
		CAString(){;}
		CAString(const wchar_t src[], size_t srcSize = 0);
		CAString(const char src[], size_t srcSize = 0);
		CAString(const CAString& src);

		~CAString(){
			reset();
		}

		typedef char UNIT;

		size_t getInputLength(const char src[]) const;
		size_t getInputLengthW(const wchar_t src[]) const;

		void copyFromW(const wchar_t src[], size_t srcSize = 0);

		void appendFromW(const wchar_t src[], size_t srcSize = 0);
		void appendFromW(const wchar_t wch);


		int  decompose(const char delimiter, std::vector<CAString>& lstWords) const;
		bool lcompare(const char chrBuffer[], size_t bufSize = 0) const;


		//Useful Funcs
		CAString substr(size_t offset, size_t count = -1) const;
		CAString& toUpper();
		CAString& toLower();
		CAString& trim();
		CAString& removeStartEndSpaces();

		friend bool operator==(const CAString& a, const CAString& b);	
		friend bool operator!=(const CAString& a, const CAString& b);	
		friend std::ostream& operator <<(std::ostream& outs, const CAString& src);
		friend std::istream& operator >>(std::istream& ins, CAString& src);

		CAString operator +(const CAString& src) const;
		CAString operator +(const wchar_t src[]) const;
		CAString operator +(wchar_t src[]) const;
		CAString operator +(const char src[]) const;
		CAString operator +(char src[]) const;
		CAString operator +(const char ch) const;	
		CAString operator +(const wchar_t ch) const;	

		void operator +=(const CAString& src);
		void operator +=(const wchar_t src[]);
		void operator +=(wchar_t src[]);
		void operator +=(const char src[]);
		void operator +=(char src[]);
		void operator +=(const char ch);
		void operator +=(const wchar_t ch);



		void operator=(const char src[]);
		void operator=(const wchar_t src[]);
	};


	//Wide String to represent UniCode String characters
	class CWString : public CTString<wchar_t>
	{	
	public:	
		CWString(){;}
		CWString(const char src[], size_t srcSize = 0);
		CWString(const wchar_t src[], size_t srcSize = 0);
		CWString(const CWString& src);

		~CWString(){ reset();}


		typedef wchar_t UNIT;
		size_t getInputLength(const wchar_t src[]) const;


		void copyFromA(const char src[], size_t srcSize = 0);

		void appendFromA(const char src[], size_t srcSize = 0);
		void appendFromA(const char ch);

		//Useful Functions
		CWString substr(size_t offset, size_t count = -1) const;
		CWString& toUpper();
		CWString& toLower();


		//operators
		friend bool operator==(const CWString& a, const CWString& b);	
		friend bool operator!=(const CWString& a, const CWString& b);
		friend std::ostream& operator <<(std::ostream& outs, const CWString& src);
		friend std::istream& operator >>(std::istream& ins, CWString& src);

		CWString operator +(const CWString& src) const;
		CWString operator +(const wchar_t src[]) const;
		CWString operator +(wchar_t src[]) const;
		CWString operator +(const char src[]) const;
		CWString operator +(char src[]) const;
		CWString operator +(const char ch) const;	
		CWString operator +(const wchar_t ch) const;	

		void operator +=(const CWString& src);
		void operator +=(const wchar_t src[]);
		void operator +=(wchar_t src[]);
		void operator +=(const char src[]);
		void operator +=(char src[]);
		void operator +=(const char ch);
		void operator +=(const wchar_t ch);


		void operator=(const char src[]);
		void operator=(const wchar_t src[]);
	};

	//Useful for conversion between the two
	CAString toAnsiString(const CWString& strWide);
	CWString toWideString(const CAString& strAnsi);
	CAString printToAStr( const char *pFmt, ... );
	CWString printToWStr( const char *pFmt, ... );

}

//=========================================================================================================
//TypeDefs
//=========================================================================================================
typedef PS::CAString DAnsiStr;
typedef PS::CWString DWideStr;


#endif
