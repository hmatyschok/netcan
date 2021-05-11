<pre><code> 
can(4) - Controller Area Network protocol domain(9) for FreeBSD  
================================================================

 Port of AF_CAN communication domain(9) from NetBSD 8.0: 
 
  http://cvsweb.netbsd.org/bsdweb.cgi/src/sys/netcan/?only_with_tag=MAIN
 
 There are a lot of work on my TODO list:
 
  (a) Porting of canconfig(8) and possible merge with ifconfig(8).
  
  (b) A lot of bugfixes, because this implementation is 
      not finished yet and a work in progress.
  
  (c) Operational Line-discipline based on if_slc(4). 
  
  (d) Device driver(9) for Adapter with Philips SJA1000 controller.
  
  (d) Device driver(9) for Adapter with Bosh C_CAN controller.
  
  (f)  ...   
 
Legal Notice: 
-------------
 
  (a) NetBSD is a registered trademark of the NetBSD Foundation.
  
  (b) FreeBSD is a registered trademark of the FreeBSD Foundation. 
  
  (c) PHILIPS is a registered trademark of Koninklijke Philips N.V.
  
  (d) BOSCH is a registered trademark of Robert Bosch GmbH.
  
  (e) Don't use this software on production systems!
</code></pre>

