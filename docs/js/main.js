// ============================================
// Indecisive Devices Website - Main JS
// ============================================

// Initialize AOS (Animate On Scroll)
document.addEventListener('DOMContentLoaded', function() {
    AOS.init({
        duration: 800,
        easing: 'ease-in-out',
        once: true,
        offset: 100
    });

    // Initialize contact form
    initContactForm();
    
    // Add smooth scroll behavior for navigation links
    initSmoothScroll();
    
    // Update active nav link on scroll
    initActiveNavLink();
});

// ============================================
// CONTACT FORM HANDLING
// ============================================

function initContactForm() {
    const form = document.getElementById('contact-form');
    
    if (form) {
        form.addEventListener('submit', function(e) {
            e.preventDefault();
            
            // Get form data
            const name = document.getElementById('name').value;
            const email = document.getElementById('email').value;
            const subject = document.getElementById('subject').value;
            const message = document.getElementById('message').value;
            
            // Validate form
            if (!name || !email || !subject || !message) {
                showFormAlert('Please fill out all fields.', 'danger');
                return;
            }
            
            // Create FormData for submission
            const formData = new FormData();
            formData.append('name', name);
            formData.append('email', email);
            formData.append('subject', subject);
            formData.append('message', message);
            
            // Show loading state
            const submitBtn = form.querySelector('button[type="submit"]');
            const originalText = submitBtn.textContent;
            submitBtn.disabled = true;
            submitBtn.textContent = 'Sending...';
            
            // For now, simulate form submission
            // Replace this with actual FormSpree endpoint or your backend
            setTimeout(function() {
                showFormAlert('Thank you for your message! We\'ll be in touch soon.', 'success');
                form.reset();
                submitBtn.disabled = false;
                submitBtn.textContent = originalText;
            }, 1000);
            
            /* 
            // Uncomment below when you have a FormSpree endpoint
            fetch('https://formspree.io/f/YOUR_FORM_ID', {
                method: 'POST',
                body: formData,
                headers: {
                    'Accept': 'application/json'
                }
            })
            .then(response => {
                if (response.ok) {
                    showFormAlert('Thank you for your message! We\'ll be in touch soon.', 'success');
                    form.reset();
                } else {
                    showFormAlert('Oops! There was an error. Please try again.', 'danger');
                }
                submitBtn.disabled = false;
                submitBtn.textContent = originalText;
            })
            .catch(error => {
                showFormAlert('Oops! There was an error. Please try again.', 'danger');
                submitBtn.disabled = false;
                submitBtn.textContent = originalText;
            });
            */
        });
    }
}

function showFormAlert(message, type) {
    // Remove existing alerts
    const existingAlert = document.querySelector('.form-alert');
    if (existingAlert) {
        existingAlert.remove();
    }
    
    // Create alert element
    const alertDiv = document.createElement('div');
    alertDiv.className = `alert alert-${type} form-alert`;
    alertDiv.setAttribute('role', 'alert');
    alertDiv.textContent = message;
    
    // Insert before form
    const form = document.getElementById('contact-form');
    form.parentNode.insertBefore(alertDiv, form);
    
    // Remove alert after 5 seconds
    setTimeout(function() {
        alertDiv.remove();
    }, 5000);
}

// ============================================
// SMOOTH SCROLL FOR NAVIGATION
// ============================================

function initSmoothScroll() {
    const navLinks = document.querySelectorAll('.navbar-nav a, a[href^="#"]');
    
    navLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            const href = this.getAttribute('href');
            
            // Only handle same-page links
            if (href.startsWith('#') && href !== '#') {
                e.preventDefault();
                
                const targetId = href.substring(1);
                const targetElement = document.getElementById(targetId);
                
                if (targetElement) {
                    // Close mobile menu if open
                    const navbarCollapse = document.querySelector('.navbar-collapse');
                    if (navbarCollapse.classList.contains('show')) {
                        const closeButton = document.querySelector('.navbar-toggler');
                        closeButton.click();
                    }
                    
                    // Scroll to target with offset for fixed navbar
                    const offsetTop = targetElement.offsetTop - 60;
                    window.scrollTo({
                        top: offsetTop,
                        behavior: 'smooth'
                    });
                }
            }
        });
    });
}

// ============================================
// ACTIVE NAV LINK ON SCROLL
// ============================================

function initActiveNavLink() {
    const sections = document.querySelectorAll('section[id]');
    const navLinks = document.querySelectorAll('.navbar-nav a[href^="#"]');
    
    window.addEventListener('scroll', function() {
        let current = '';
        
        sections.forEach(section => {
            const sectionTop = section.offsetTop;
            const sectionHeight = section.clientHeight;
            
            if (window.pageYOffset >= sectionTop - 100) {
                current = section.getAttribute('id');
            }
        });
        
        navLinks.forEach(link => {
            link.classList.remove('active');
            
            if (link.getAttribute('href').substring(1) === current) {
                link.classList.add('active');
            }
        });
    });
}

// ============================================
// PARALLAX EFFECT (Optional Enhancement)
// ============================================

function initParallax() {
    const parallaxElements = document.querySelectorAll('[data-parallax]');
    
    if (parallaxElements.length === 0) return;
    
    window.addEventListener('scroll', function() {
        parallaxElements.forEach(element => {
            const scrollPosition = window.pageYOffset;
            const speed = element.getAttribute('data-parallax') || 0.5;
            
            element.style.transform = `translateY(${scrollPosition * speed}px)`;
        });
    });
}

// ============================================
// LAZY LOADING IMAGES (Optional)
// ============================================

function initLazyLoading() {
    // Use native lazy loading or Intersection Observer
    const images = document.querySelectorAll('img[data-src]');
    
    if ('IntersectionObserver' in window) {
        const imageObserver = new IntersectionObserver((entries, observer) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    const img = entry.target;
                    img.src = img.getAttribute('data-src');
                    img.removeAttribute('data-src');
                    observer.unobserve(img);
                }
            });
        });
        
        images.forEach(img => imageObserver.observe(img));
    } else {
        // Fallback for older browsers
        images.forEach(img => {
            img.src = img.getAttribute('data-src');
            img.removeAttribute('data-src');
        });
    }
}

// ============================================
// SCROLL TO TOP BUTTON
// ============================================

function initScrollToTop() {
    // Create scroll to top button
    const scrollToTopBtn = document.createElement('button');
    scrollToTopBtn.id = 'scrollToTopBtn';
    scrollToTopBtn.className = 'scroll-to-top';
    scrollToTopBtn.setAttribute('aria-label', 'Scroll to top');
    scrollToTopBtn.textContent = '↑';
    
    document.body.appendChild(scrollToTopBtn);
    
    // Add styles
    const style = document.createElement('style');
    style.textContent = `
        .scroll-to-top {
            position: fixed;
            bottom: 30px;
            right: 30px;
            width: 50px;
            height: 50px;
            background-color: #0CA598;
            color: white;
            border: none;
            border-radius: 50%;
            cursor: pointer;
            display: none;
            justify-content: center;
            align-items: center;
            font-size: 1.5rem;
            z-index: 999;
            transition: all 0.3s ease;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
        }
        
        .scroll-to-top:hover {
            background-color: #0a8a7a;
            transform: translateY(-5px);
            box-shadow: 0 8px 16px rgba(0, 0, 0, 0.2);
        }
        
        .scroll-to-top.show {
            display: flex;
        }
        
        @media (max-width: 576px) {
            .scroll-to-top {
                bottom: 20px;
                right: 20px;
                width: 45px;
                height: 45px;
            }
        }
    `;
    document.head.appendChild(style);
    
    // Show/hide button on scroll
    window.addEventListener('scroll', function() {
        if (window.pageYOffset > 300) {
            scrollToTopBtn.classList.add('show');
        } else {
            scrollToTopBtn.classList.remove('show');
        }
    });
    
    // Scroll to top on click
    scrollToTopBtn.addEventListener('click', function() {
        window.scrollTo({
            top: 0,
            behavior: 'smooth'
        });
    });
}

// Initialize scroll to top when DOM is ready
document.addEventListener('DOMContentLoaded', initScrollToTop);

// ============================================
// COUNTER ANIMATION (For Stats, if needed)
// ============================================

function animateCounter(element, target, duration = 2000) {
    const start = 0;
    const increment = target / (duration / 16);
    let current = start;
    
    const counter = setInterval(() => {
        current += increment;
        if (current >= target) {
            element.textContent = target;
            clearInterval(counter);
        } else {
            element.textContent = Math.floor(current);
        }
    }, 16);
}

// ============================================
// RESPONSIVE NAVBAR ACTIVE STATE
// ============================================

function handleNavbarResponsive() {
    const navbarToggler = document.querySelector('.navbar-toggler');
    const navbarCollapse = document.querySelector('.navbar-collapse');
    
    if (navbarToggler) {
        navbarToggler.addEventListener('click', function() {
            // This is handled by Bootstrap, but you can add custom behavior here
        });
    }
}

// Initialize on DOM ready
document.addEventListener('DOMContentLoaded', handleNavbarResponsive);

// ============================================
// UTILITY FUNCTION: Debounce
// ============================================

function debounce(func, delay) {
    let timeoutId;
    return function(...args) {
        clearTimeout(timeoutId);
        timeoutId = setTimeout(() => func(...args), delay);
    };
}

// ============================================
// UTILITY FUNCTION: Throttle
// ============================================

function throttle(func, limit) {
    let inThrottle;
    return function(...args) {
        if (!inThrottle) {
            func.apply(this, args);
            inThrottle = true;
            setTimeout(() => inThrottle = false, limit);
        }
    };
}

// ============================================
// LOG INITIALIZATION
// ============================================

console.log('Indecisive Devices website loaded successfully!');
