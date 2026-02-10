# Indecisive Devices GitHub Pages Website

Welcome to the official GitHub Pages website for the Indecisive Devices FTC team!

## Overview

This is a modern, responsive Single Page Application (SPA) hosted on GitHub Pages, designed to showcase our team, robots, and journey in FIRST Tech Challenge.

## Live Website

Your website will be hosted at: `https://IndecisiveDevices.github.io/FtcRobotController/`

## Project Structure

```
docs/
├── index.html          # Main HTML file (the website)
├── css/
│   └── style.css       # Custom styling with team colors
├── js/
│   └── main.js         # JavaScript for interactivity and animations
└── assets/             # (For future images, videos, and media)
```

## Features

✨ **Modern Design**
- Responsive layout that works on desktop, tablet, and mobile
- Professional gradient backgrounds and smooth animations
- Team color scheme (Teal #0CA598, Yellow #C7BB39, Red #C23E46)

📱 **Sections Included**
1. **Hero** - Eye-catching introduction with call-to-action buttons
2. **About Us** - Team history and FIRST introduction
3. **Our Robots** - Showcase of current and past robots
4. **Sponsors** - Tiered sponsor display (Platinum, Gold, Silver)
5. **Get Involved** - Ways to join, mentor, and sponsor
6. **Contact Form** - Get in touch with the team
7. **Donations** - Explain the mission and link to donation platforms
8. **Footer** - Social media links and quick navigation

🎨 **Design Highlights**
- Custom CSS with BEM methodology for maintainability
- Bootstrap 5 for responsive grid and components
- Google Fonts (Montserrat headings, Lato body text)
- AOS (Animate On Scroll) for smooth scroll animations
- Parallax effects and hover transitions

🔧 **Functionality**
- Smooth scrolling navigation
- Active nav link highlighting
- Contact form validation (ready for FormSpree integration)
- Scroll-to-top button
- Mobile-responsive navbar

## Getting Started

### View the Website Locally

1. Clone the repository:
   ```bash
   git clone https://github.com/IndecisiveDevices/FtcRobotController.git
   cd FtcRobotController
   ```

2. Open the website locally:
   - **Option A**: Simply open `docs/index.html` in your web browser
   - **Option B**: Use a local server (recommended for testing):
     ```bash
     # Using Python 3
     python3 -m http.server 8000 --directory docs
     
     # Or using Node.js (if you have it)
     npx serve docs
     ```
   - Then visit `http://localhost:8000` in your browser

### Deploy to GitHub Pages

1. **Ensure GitHub Pages is enabled**:
   - Go to your repository settings
   - Scroll to "Pages" section
   - Set source to "Deploy from a branch"
   - Set branch to `add-github-page` (or your main branch)
   - Set folder to `/docs`
   - Save

2. **Push changes to trigger deployment**:
   ```bash
   git add .
   git commit -m "Add GitHub Pages website"
   git push origin add-github-page
   ```

3. Your website will be live at: `https://IndecisiveDevices.github.io/FtcRobotController/`

## Customization Guide

### Update Team Information

1. **Site Title & Meta**: Edit the `<title>` and meta tags in `index.html`
2. **Hero Section**: Update hero text, images, and CTA buttons
3. **About Section**: Add team history, founding year, and mission
4. **Robot Details**: Replace placeholders with actual robot specifications
5. **Sponsor Logos**: Add sponsor images to `assets/` folder and update image paths
6. **Contact Email**: Update `mailto` links throughout the site

### Update Colors

Edit the CSS variables in `docs/css/style.css`:
```css
:root {
    --primary: #0CA598;      /* Teal */
    --secondary: #C7BB39;    /* Yellow */
    --accent: #C23E46;       /* Red */
    --dark: #1A1A1A;         /* Dark */
    --light: #F8F9FA;        /* Light */
}
```

### Add Images

1. Create folders in `docs/assets/` (e.g., `images/`, `videos/`)
2. Place your images there
3. Update image paths in `index.html`:
   ```html
   <img src="assets/images/your-image.jpg" alt="Description">
   ```

### Integrate Contact Form

To enable real contact form submissions, sign up for [Formspree](https://formspree.io/):

1. Create a form at Formspree and get your form ID
2. Uncomment the FormSpree fetch code in `docs/js/main.js`
3. Replace `YOUR_FORM_ID` with your actual Formspree ID
4. Test the form

### Add Social Media Links

Update the social icons in the footer section of `index.html`:
```html
<a href="https://facebook.com/your-page" class="social-icon">📘</a>
<a href="https://twitter.com/your-handle" class="social-icon">🐦</a>
<a href="https://instagram.com/your-profile" class="social-icon">📷</a>
<a href="https://youtube.com/your-channel" class="social-icon">▶️</a>
```

## Placeholder Content

All placeholder content is clearly marked with `[Placeholder - description]` in the code. Replace these with actual content:

- Team photos and robot images
- Robot specifications and features
- Sponsor logos
- Team member information
- Contact email addresses
- Donation platform links

## Browser Compatibility

- Chrome/Edge (latest)
- Firefox (latest)
- Safari (latest)
- Mobile browsers (iOS Safari, Chrome Mobile)

## Dependencies

All dependencies are loaded via CDN, so no installation required:
- **Bootstrap 5.3.0** - CSS framework
- **Google Fonts** - Typography
- **AOS 2.3.1** - Scroll animations

## File Sizes

- `index.html` - ~25 KB
- `css/style.css` - ~20 KB
- `js/main.js` - ~8 KB
- **Total**: ~53 KB (very lightweight)

## Performance Tips

1. **Optimize Images**: Use compressed images and WebP format
   - Use tools like [TinyPNG](https://tinypng.com/) or [ImageOptim](https://imageoptim.com/)

2. **Lazy Load Images**: Add `data-src` attribute and uncomment `initLazyLoading()` in `main.js`

3. **Minimize CSS/JS**: For production, minify files using tools like:
   - [CSS Minifier](https://cssminifier.com/)
   - [JavaScript Minifier](https://javascript-minifier.com/)

## SEO Best Practices

- ✅ Meta tags for description and author
- ✅ Proper heading hierarchy (H1, H2, H3)
- ✅ Descriptive image alt text
- ✅ Mobile-responsive design
- 🔲 To add: Structured data (Schema.org markup)
- 🔲 To add: Sitemap and robots.txt

## Accessibility

- ✅ Semantic HTML structure
- ✅ ARIA labels on interactive elements
- ✅ Proper color contrast ratios
- ✅ Keyboard navigation support

## Future Enhancements

Ideas for improving the website:
- [ ] Add photo/video gallery with lightbox
- [ ] Create a blog/news section for team updates
- [ ] Add team member profiles
- [ ] Integrate with social media feeds
- [ ] Add event calendar
- [ ] Create achievement badges/timeline
- [ ] Add testimonials section
- [ ] Implement dark mode toggle
- [ ] Add live competition updates

## License

© 2024-2026 Indecisive Devices. All rights reserved.

## Support

For questions or issues with the website, please:
1. Check the code comments in `index.html`, `style.css`, and `main.js`
2. Review the customization guide above
3. Contact the team lead for help

---

**Ready to customize?** Start by updating the hero section and adding your team's photos! Good luck with the competition! 🚀

