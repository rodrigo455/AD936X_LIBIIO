AC_INIT(AD936X_LIBIIO, 2.0.2)
AM_INIT_AUTOMAKE([nostdinc foreign])
AC_CONFIG_MACRO_DIR([m4])

AC_PROG_CC
AC_PROG_CXX
AC_PROG_INSTALL

AC_CORBA_ORB
OSSIE_CHECK_OSSIE
OSSIE_SDRROOT_AS_PREFIX

AC_ARG_WITH(domainname,
        AS_HELP_STRING([--with-domainname=[domain name]], [Provide a domain name to the generated node, defaults to REDHAWK_DEV]),
        ad936x_libiio_node_domain=$withval,
        usrp_uhd_node_domain=REDHAWK_DEV)
AC_SUBST(DOMAINNAME, $ad936x_libiio_node_domain)

AC_ARG_WITH(ad936xtype,
        AS_HELP_STRING([--with-ad936xtype=[ad936x type]], [Provide a AD936X type to the generated node, defaults to ad9361]),
        ad936x_libiio_node_type=$withval,
        ad936x_libiio_node_type=ad9361)
AC_SUBST(AD936XTYPE, $ad936x_libiio_node_type)

AC_ARG_WITH(ad936xcontexturi,
        AS_HELP_STRING([--with-ad936xcontexturi=[iio context uri]], [Provide a iio context URI to the generated node, defaults to '']),
        ad936x_libiio_uri=$withval,
        ad936x_libiio_uri='')
AC_SUBST(CONTEXTURI, $ad936x_libiio_uri)

m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

# Dependencies
export PKG_CONFIG_PATH="$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig"
PKG_CHECK_MODULES([PROJECTDEPS], [ossie >= 2.0 omniORB4 >= 4.1.0])
PKG_CHECK_MODULES([INTERFACEDEPS], [frontend >= 2.3, bulkio >= 2.0])
PKG_CHECK_MODULES([AD936XDEPS], [libiio >= 0.8 libad9361])
PKG_CHECK_MODULES([LIBUUID], [uuid])
OSSIE_ENABLE_LOG4CXX
AX_BOOST_BASE([1.41])
AX_BOOST_SYSTEM
AX_BOOST_THREAD
AX_BOOST_REGEX

AC_CONFIG_FILES([Makefile])
AC_OUTPUT

