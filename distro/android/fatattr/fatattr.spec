Summary: Utilities to control attributes on a FAT filesystem
Name: fatattr
Version: 1.0.1
Release: 1
License: GPL
Group: Applications/System
URL: http://www.kernel.org/pub/linux/utils/fs/fat/fatattr/
Source: http://www.kernel.org/pub/linux/utils/fs/fat/fatattr/fatattr-%{version}.tar.gz
BuildRoot: %{_tmppath}/%{name}-%{version}-root
Epoch: 1

%description
Tools to control file attributes on FAT filesystems.  FAT filesystems contain
several attributes which do not map cleanly onto Unix attributes; this
utility allow these to be controlled directly.

%prep
%setup -q 

%build
%configure
make

%install
rm -rf $RPM_BUILD_ROOT
make install INSTALLROOT="$RPM_BUILD_ROOT"

%clean
rm -rf $RPM_BUILD_ROOT

%files
%defattr(-,root,root)
%{_bindir}/fatattr
%{_mandir}/man1/fatattr.1*

%changelog
* Thu Feb 12 2009 H. Peter Anvin <hpa@zytor.com>
- Initial version.
